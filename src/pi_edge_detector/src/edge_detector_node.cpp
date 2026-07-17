#include "pi_edge_detector/edge_detector_node.hpp"

#include <chrono>
#include <functional>

namespace pi_edge_detector {

EdgeDetectorNode::EdgeDetectorNode(const rclcpp::NodeOptions & options)
  : Node("edge_detector", options),
    perf_monitor_(100),
    last_process_time_(0, 0, RCL_ROS_TIME)
{
  // Declare parameters
  this->declare_parameter<std::string>("algorithm", "canny");
  this->declare_parameter<int>("canny_low_threshold", 50);
  this->declare_parameter<int>("canny_high_threshold", 150);
  this->declare_parameter<int>("sobel_kernel_size", 3);
  this->declare_parameter<int>("target_width", 640);
  this->declare_parameter<double>("max_fps", 15.0);
  this->declare_parameter<bool>("publish_diagnostics", true);

  // Retrieve parameters
  algorithm_ = this->get_parameter("algorithm").as_string();
  edge_params_.canny_low = this->get_parameter("canny_low_threshold").as_int();
  edge_params_.canny_high = this->get_parameter("canny_high_threshold").as_int();
  edge_params_.sobel_ksize = this->get_parameter("sobel_kernel_size").as_int();
  target_width_ = this->get_parameter("target_width").as_int();
  max_fps_ = this->get_parameter("max_fps").as_double();
  publish_diagnostics_ = this->get_parameter("publish_diagnostics").as_bool();

  // Initialize publishers and subscribers
  image_pub_ = image_transport::create_publisher(this, "output/edges");
  image_sub_ = image_transport::create_subscription(
    this, "input/image_raw",
    std::bind(&EdgeDetectorNode::image_callback, this, std::placeholders::_1),
    "raw"
  );
  
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diag_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&EdgeDetectorNode::diagnostics_callback, this)
  );

  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&EdgeDetectorNode::on_parameter_change, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Edge detector ready [algorithm=%s, max_fps=%.1f]",
              algorithm_.c_str(), max_fps_);
}

void EdgeDetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // FPS Throttling
  auto now = this->now();
  double elapsed = (now - last_process_time_).seconds();
  double min_interval = 1.0 / max_fps_;
  if (elapsed < min_interval) {
    perf_monitor_.record_drop();
    return;
  }
  last_process_time_ = now;
  
  // Convert ROS Image to OpenCV Mat without copying if possible
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
  } catch(const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
    return;
  }
  const cv::Mat& input = cv_ptr->image;

  // Processing pipeline
  auto start = std::chrono::steady_clock::now();

  if (target_width_ > 0 && target_width_ < input.cols) {
    double scale = static_cast<double>(target_width_) / input.cols;
    cv::resize(input, resize_buffer_, cv::Size(), scale, scale, cv::INTER_LINEAR);
    cv::cvtColor(resize_buffer_, gray_buffer_, cv::COLOR_BGR2GRAY);
  } else {
    cv::cvtColor(input, gray_buffer_, cv::COLOR_BGR2GRAY);
  }

  // Detect edges
  EdgeAlgorithms::detect(algorithm_, gray_buffer_, edge_buffer_, edge_params_);

  auto end = std::chrono::steady_clock::now();
  double latency = std::chrono::duration<double, std::milli>(end - start).count();
  perf_monitor_.record_frame(latency);

  // Publish output image
  auto out_msg = cv_bridge::CvImage(msg->header, "mono8", edge_buffer_).toImageMsg();
  image_pub_.publish(*out_msg);
}

void EdgeDetectorNode::diagnostics_callback()
{
  if (!publish_diagnostics_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Perf: %s", perf_monitor_.get_summary().c_str());

  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "edge_detector";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = perf_monitor_.get_summary();

  // Add metrics key-values
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
EdgeDetectorNode::on_parameter_change(const std::vector<rclcpp::Parameter> & params)
{
  for (const auto& p : params) {
    if (p.get_name() == "algorithm") {
      algorithm_ = p.as_string();
    } else if (p.get_name() == "canny_low_threshold") {
      edge_params_.canny_low = p.as_int();
    } else if (p.get_name() == "canny_high_threshold") {
      edge_params_.canny_high = p.as_int();
    } else if (p.get_name() == "sobel_kernel_size") {
      edge_params_.sobel_ksize = p.as_int();
    } else if (p.get_name() == "max_fps") {
      max_fps_ = p.as_double();
    } else if (p.get_name() == "target_width") {
      target_width_ = p.as_int();
    }
  }

  RCLCPP_INFO(this->get_logger(), "Parameters updated: [algorithm=%s, target_width=%d, max_fps=%.1f]",
              algorithm_.c_str(), target_width_, max_fps_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace pi_edge_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pi_edge_detector::EdgeDetectorNode)
