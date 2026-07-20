#include "pi_yolov26_detector/yolov26_detector_node.hpp"

#include <chrono>
#include <functional>
#include <fstream>
#include <algorithm>

namespace pi_yolov26_detector {

Yolov26DetectorNode::Yolov26DetectorNode(const rclcpp::NodeOptions & options)
  : Node("yolov26_detector", options),
    perf_monitor_(100),
    last_process_time_(0, 0, RCL_ROS_TIME)
{
  // Declare parameters
  this->declare_parameter<std::string>("model_path", "models/yolov8n.onnx");
  this->declare_parameter<double>("confidence_threshold", 0.25);
  this->declare_parameter<int>("target_width", 640);
  this->declare_parameter<double>("max_fps", 10.0);
  this->declare_parameter<bool>("publish_diagnostics", true);

  // Retrieve parameters
  model_path_ = this->get_parameter("model_path").as_string();
  confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
  target_width_ = this->get_parameter("target_width").as_int();
  max_fps_ = this->get_parameter("max_fps").as_double();
  publish_diagnostics_ = this->get_parameter("publish_diagnostics").as_bool();

  // Load the network
  load_model();

  // Load default class names (COCO classes)
  // Feel free to modify this or load class names from a text file parameter
  class_names_ = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush"
  };

  // Initialize publishers and subscribers
  image_pub_ = image_transport::create_publisher(this, "output/detections");
  image_sub_ = image_transport::create_subscription(
    this, "input/image_raw",
    std::bind(&Yolov26DetectorNode::image_callback, this, std::placeholders::_1),
    "raw"
  );
  
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diag_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&Yolov26DetectorNode::diagnostics_callback, this)
  );

  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&Yolov26DetectorNode::on_parameter_change, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "YOLOv26 detector ready [model=%s, max_fps=%.1f]",
              model_path_.c_str(), max_fps_);
}

void Yolov26DetectorNode::load_model()
{
  try {
    RCLCPP_INFO(this->get_logger(), "Loading ONNX model from: %s", model_path_.c_str());
    
    net_ = cv::dnn::readNetFromONNX(model_path_);
    // CPU Optimizations:
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    
    RCLCPP_INFO(this->get_logger(), "Model loaded successfully!");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load model: %s. Check the path and format.", e.what());
  }
}

void Yolov26DetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // 1. FPS Throttling
  auto now = this->now();
  double elapsed = (now - last_process_time_).seconds();
  double min_interval = 1.0 / max_fps_;
  if (elapsed < min_interval) {
    perf_monitor_.record_drop();
    return;
  }
  last_process_time_ = now;
  
  // 2. Convert ROS Image to OpenCV Mat
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch(const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
    return;
  }
  
  // We make a copy to annotate on it
  output_buffer_ = cv_ptr->image.clone();

  // 3. Inference Pipeline
  auto start = std::chrono::steady_clock::now();

  // Preprocess: Convert output_buffer_ to 640x640 RGB blob scaled by 1/255.0
  cv::dnn::blobFromImage(output_buffer_, blob_buffer_, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(0, 0, 0), true, false);
  
  // Set the input to the network
  net_.setInput(blob_buffer_);
  
  // Run forward pass
  std::vector<cv::Mat> outputs;
  net_.forward(outputs, net_.getUnconnectedOutLayersNames());
  
  // Parse outputs if we received them
  if (!outputs.empty()) {
    cv::Mat output = outputs[0]; // Shape: [1, 84, 8400]
    
    int dimensions = output.size[1]; // 84
    int rows = output.size[2];       // 8400
    cv::Mat raw_data(dimensions, rows, CV_32F, output.ptr<float>());
    cv::Mat detections;
    cv::transpose(raw_data, detections); // Physical transpose makes memory contiguous
    
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;
    
    float scale_x = static_cast<float>(output_buffer_.cols) / 640.0f;
    float scale_y = static_cast<float>(output_buffer_.rows) / 640.0f;
    
    for (int i = 0; i < detections.rows; ++i) {
      cv::Mat row = detections.row(i);
      cv::Mat scores = row.colRange(4, 84); // Class scores
      
      cv::Point class_id_point;
      double max_class_score;
      cv::minMaxLoc(scores, nullptr, &max_class_score, nullptr, &class_id_point);
      
      if (max_class_score > confidence_threshold_) {
        float cx = row.at<float>(0);
        float cy = row.at<float>(1);
        float w = row.at<float>(2);
        float h = row.at<float>(3);
        
        int left = static_cast<int>((cx - 0.5f * w) * scale_x);
        int top = static_cast<int>((cy - 0.5f * h) * scale_y);
        int width = static_cast<int>(w * scale_x);
        int height = static_cast<int>(h * scale_y);
        
        // Boundaries checks
        left = std::max(0, left);
        top = std::max(0, top);
        width = std::min(output_buffer_.cols - left, width);
        height = std::min(output_buffer_.rows - top, height);
        
        if (width > 0 && height > 0) {
          boxes.push_back(cv::Rect(left, top, width, height));
          confidences.push_back(static_cast<float>(max_class_score));
          class_ids.push_back(class_id_point.x);
        }
      }
    }
    
    // Apply Non-Maximum Suppression (NMS)
    std::vector<int> indices;
    float nms_threshold = 0.45f;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, nms_threshold, indices);
    
    // Draw bounding boxes and labels
    for (int idx : indices) {
      cv::Rect box = boxes[idx];
      int class_id = class_ids[idx];
      float confidence = confidences[idx];
      
      cv::rectangle(output_buffer_, box, cv::Scalar(0, 255, 0), 2);
      
      std::string label = "";
      if (class_id < static_cast<int>(class_names_.size())) {
        label = class_names_[class_id];
      } else {
        label = "class_" + std::to_string(class_id);
      }
      label += ": " + cv::format("%.2f", confidence);
      
      int baseLine;
      cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
      int label_y = std::max(box.y, label_size.height + 5);
      
      cv::rectangle(output_buffer_, 
                    cv::Point(box.x, label_y - label_size.height - 5),
                    cv::Point(box.x + label_size.width, label_y + baseLine - 5),
                    cv::Scalar(0, 255, 0), cv::FILLED);
                    
      cv::putText(output_buffer_, label, cv::Point(box.x, label_y - 5), 
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
  }

  auto end = std::chrono::steady_clock::now();
  double latency = std::chrono::duration<double, std::milli>(end - start).count();
  perf_monitor_.record_frame(latency);

  // 4. Publish annotated output image
  auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", output_buffer_).toImageMsg();
  image_pub_.publish(*out_msg);
}

void Yolov26DetectorNode::diagnostics_callback()
{
  if (!publish_diagnostics_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Perf: %s", perf_monitor_.get_summary().c_str());

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "yolov26_detector";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = perf_monitor_.get_summary();

  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "avg_latency_ms";
  kv.value = std::to_string(perf_monitor_.avg_latency_ms());
  status.values.push_back(kv);

  kv.key = "measured_fps";
  kv.value = std::to_string(perf_monitor_.measured_fps());
  status.values.push_back(kv);

  kv.key = "total_frames";
  kv.value = std::to_string(perf_monitor_.total_frames());
  status.values.push_back(kv);

  kv.key = "total_drops";
  kv.value = std::to_string(perf_monitor_.total_drops());
  status.values.push_back(kv);

  diag_array.status.push_back(status);
  diag_pub_->publish(diag_array);
}

rcl_interfaces::msg::SetParametersResult
Yolov26DetectorNode::on_parameter_change(const std::vector<rclcpp::Parameter> & params)
{
  bool reload_required = false;
  for (const auto& p : params) {
    if (p.get_name() == "model_path") {
      model_path_ = p.as_string();
      reload_required = true;
    } else if (p.get_name() == "confidence_threshold") {
      confidence_threshold_ = p.as_double();
    } else if (p.get_name() == "max_fps") {
      max_fps_ = p.as_double();
    } else if (p.get_name() == "target_width") {
      target_width_ = p.as_int();
    }
  }

  if (reload_required) {
    load_model();
  }

  RCLCPP_INFO(this->get_logger(), "Parameters updated: [model=%s, confidence=%.2f, max_fps=%.1f]",
              model_path_.c_str(), confidence_threshold_, max_fps_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace pi_yolov26_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pi_yolov26_detector::Yolov26DetectorNode)
