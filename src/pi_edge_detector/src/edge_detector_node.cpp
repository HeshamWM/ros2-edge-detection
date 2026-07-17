#include "pi_edge_detector/edge_detector_node.hpp"

#include <chrono>
#include <functional>

namespace pi_edge_detector {

// =============================================================================
// TASK 1: Implement the constructor
// =============================================================================
// This is the most complex part. Follow these steps:
//
// Step 1a — Call the parent constructor:
//   Use the initializer list to call:
//     Node("edge_detector", options)
//   Also initialize perf_monitor_ and last_process_time_ (to this->now()).
//
// Step 1b — Declare ROS 2 parameters:
//   Use this->declare_parameter<T>("name", default_value) for each:
//     - "algorithm"            (string,  "canny")
//     - "canny_low_threshold"  (int,     50)
//     - "canny_high_threshold" (int,     150)
//     - "sobel_kernel_size"    (int,     3)
//     - "target_width"         (int,     640)
//     - "max_fps"              (double,  15.0)
//     - "publish_diagnostics"  (bool,    true)
//
// Step 1c — Read declared parameters into member variables:
//   algorithm_            = this->get_parameter("algorithm").as_string();
//   edge_params_.canny_low = this->get_parameter("canny_low_threshold").as_int();
//   ... (do this for all parameters)
//
// Step 1d — Set up image_transport subscriber and publisher:
//   image_pub_ = image_transport::create_publisher(this, "output/edges");
//   image_sub_ = image_transport::create_subscription(
//       this, "input/image_raw",
//       std::bind(&EdgeDetectorNode::image_callback, this, std::placeholders::_1),
//       "raw"    // transport hint
//   );
//
// Step 1e — Set up diagnostics publisher and timer:
//   diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
//       "/diagnostics", 10);
//   diag_timer_ = this->create_wall_timer(
//       std::chrono::seconds(1),
//       std::bind(&EdgeDetectorNode::diagnostics_callback, this));
//
// Step 1f — Register the parameter change callback:
//   param_cb_handle_ = this->add_on_set_parameters_callback(
//       std::bind(&EdgeDetectorNode::on_parameter_change, this, std::placeholders::_1));
//
// Step 1g — Log that the node is ready:
//   RCLCPP_INFO(this->get_logger(), "Edge detector ready [algorithm=%s, max_fps=%.1f]",
//               algorithm_.c_str(), max_fps_);
// =============================================================================
EdgeDetectorNode::EdgeDetectorNode(const rclcpp::NodeOptions & options)
  : Node("edge_detector", options), perf_monitor_(100), last_process_time_(0, 0, RCL_ROS_TIME)
{
  this->declare_parameter<std::string>("algorithm", "canny");
  this->declare_parameter<int>("canny_low_threshold", 50);
  this->declare_parameter<int>("canny_high_threshold", 150);
  this->declare_parameter<int>("sobel_kernel_size", 3);
  this->declare_parameter<int>("target_width", 640);
  this->declare_parameter<double>("max_fps", 15.0);
  this->declare_parameter<bool>("publish_diagnostics", true);

  algorithm_ = this->get_parameter("algorithm").as_string();
  edge_params_.canny_low = this->get_parameter("canny_low_threshold").as_int();
  edge_params_.canny_high = this->get_parameter("canny_high_threshold").as_int();
  edge_params_.sobel_ksize = this->get_parameter("sobel_kernel_size").as_int();
  target_width_ = this->get_parameter("target_width").as_int();
  max_fps_ = this->get_parameter("max_fps").as_double();
  publish_diagnostics_ = this->get_parameter("publish_diagnostics").as_bool();

  image_pub_ = image_transport::create_publisher(this, "output/edges");
  image_sub_ = image_transport::create_subscription(this, "input/image_raw",
        std::bind(&EdgeDetectorNode::image_callback, this, std::placeholders::_1), "raw");
  
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  diag_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&EdgeDetectorNode::diagnostics_callback, this));
  param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&EdgeDetectorNode::on_parameter_change, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Edge detector ready [algorithm=%s, max_fps=%.1f]",
              algorithm_.c_str(), max_fps_);
}

// =============================================================================
// TASK 2: Implement image_callback()
// =============================================================================
// This is the heart of the system. Called on every incoming frame.
//
// Step 2a — FPS throttle:
//   Compute time since last processed frame:
//     auto now = this->now();
//     double elapsed = (now - last_process_time_).seconds();
//     double min_interval = 1.0 / max_fps_;
//     if (elapsed < min_interval) {
//         perf_monitor_.record_drop();
//         return;  // skip this frame
//     }
//     last_process_time_ = now;
//
// Step 2b — Convert ROS message to OpenCV Mat:
//   Use cv_bridge to get the image WITHOUT copying if possible:
//     cv_bridge::CvImageConstPtr cv_ptr;
//     try {
//         cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
//     } catch (cv_bridge::Exception& e) {
//         RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
//         return;
//     }
//     const cv::Mat& input = cv_ptr->image;
//
// Step 2c — Start timing:
//     auto start = std::chrono::steady_clock::now();
//
// Step 2d — Preprocess (resize + grayscale):
//   If target_width_ > 0 and input.cols > target_width_:
//     - Compute scale = (double)target_width_ / input.cols
//     - cv::resize(input, resize_buffer_, cv::Size(), scale, scale, cv::INTER_LINEAR)
//     - cv::cvtColor(resize_buffer_, gray_buffer_, cv::COLOR_BGR2GRAY)
//   Else (no resize needed):
//     - cv::cvtColor(input, gray_buffer_, cv::COLOR_BGR2GRAY)
//
// Step 2e — Run edge detection:
//     EdgeAlgorithms::detect(algorithm_, gray_buffer_, edge_buffer_, edge_params_);
//
// Step 2f — Stop timing and record:
//     auto end = std::chrono::steady_clock::now();
//     double latency_ms = std::chrono::duration<double, std::milli>(end - start).count();
//     perf_monitor_.record_frame(latency_ms);
//
// Step 2g — Publish the result:
//   Convert edge_buffer_ back to a ROS message:
//     auto out_msg = cv_bridge::CvImage(
//         msg->header,         // reuse the original timestamp
//         "mono8",             // edge maps are single-channel
//         edge_buffer_
//     ).toImageMsg();
//     image_pub_.publish(*out_msg);
//
// Interview tip:
//   Be ready to explain why we use toCvShare() instead of toCvCopy().
//   Answer: toCvShare borrows the underlying buffer without copying.
//   Since we only read from it (to resize/grayscale), this is safe
//   and saves ~1ms of memcpy on a 640×480 image.
// =============================================================================
void EdgeDetectorNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // First we make sure this frame should be processed
  auto now = this->now();
  double elapsed = (now - last_process_time_ ).seconds();
  double min_interval = 1 / max_fps_;
  if (elapsed < min_interval) {
    return;
  }
  last_process_time_ = now;
  
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg);
  }
  catch(const cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge : %s", e.what());
    return;
  }
  const cv::Mat& input = cv_ptr->image;

  // Start Image processing
  auto start = std::chrono::steady_clock::now();
  if (target_width_ > 0 && target_width_ < input.cols)
  {
    double scale = static_cast<double>(target_width_) / input.cols;
    cv::resize(input, resize_buffer_, cv::Size(), scale, scale, cv::INTER_LINEAR);
    cv::cvtColor(resize_buffer_, gray_buffer_, CV_BGR2GRAY);
  } else {
    cv::cvtColor(input, gray_buffer_, CV_BGR2GRAY);
  }
  EdgeAlgorithms::detect(algorithm_, gray_buffer_, edge_buffer_, edge_params_);

  auto end = std::chrono::steady_clock::now();
  double latency = std::chrono::duration<double, std::milli>(end - start).count();
  perf_monitor_.record_frame(latency);

  auto out_msg = cv_bridge::CvImage(msg->header, "mono8", edge_buffer_).toImageMsg();
  image_pub_.publish(out_msg);
}

// =============================================================================
// TASK 3: Implement diagnostics_callback()
// =============================================================================
// Called at 1 Hz by the wall timer. Publishes a DiagnosticArray message.
//
// Steps:
//   1. If !publish_diagnostics_, return early
//   2. Log the summary:
//      RCLCPP_INFO(this->get_logger(), "Perf: %s", perf_monitor_.get_summary().c_str());
//   3. Build and publish a DiagnosticArray:
//      diagnostic_msgs::msg::DiagnosticArray diag_array;
//      diag_array.header.stamp = this->now();
//
//      diagnostic_msgs::msg::DiagnosticStatus status;
//      status.name = "edge_detector";
//      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
//      status.message = perf_monitor_.get_summary();
//
//      // Add key-value pairs
//      diagnostic_msgs::msg::KeyValue kv;
//      kv.key = "avg_latency_ms";
//      kv.value = std::to_string(perf_monitor_.avg_latency_ms());
//      status.values.push_back(kv);
//      // ... add more key-value pairs for fps, total_frames, total_drops
//
//      diag_array.status.push_back(status);
//      diag_pub_->publish(diag_array);
// =============================================================================
void EdgeDetectorNode::diagnostics_callback()
{
  if (!publish_diagnostics_) {return;}
  RCLCPP_INFO(this->get_logger(), "Perf: %s", perf_monitor_.get_summary().c_str());
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "edge_detector";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = perf_monitor_.get_summary();

  // Add key-value pairs
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "avg_latency_ms";
  kv.value = std::to_string(perf_monitor_.avg_latency_ms());
  status.values.push_back(kv);
  kv.key = "measured_fps(";
  kv.value = std::to_string(perf_monitor_.measured_fps());
  status.values.push_back(kv);

  diag_array.status.push_back(status);
  diag_pub_->publish(diag_array);
}

// =============================================================================
// TASK 4: Implement on_parameter_change()
// =============================================================================
// Called whenever someone runs `ros2 param set` on this node.
//
// Steps:
//   1. Loop through the `params` vector
//   2. For each parameter, check its name and update the corresponding member:
//      for (const auto& p : params) {
//          if (p.get_name() == "algorithm") {
//              algorithm_ = p.as_string();
//          } else if (p.get_name() == "canny_low_threshold") {
//              edge_params_.canny_low = p.as_int();
//          }
//          // ... handle all parameters
//      }
//   3. Log what changed:
//      RCLCPP_INFO(this->get_logger(), "Parameters updated");
//   4. Return a success result:
//      rcl_interfaces::msg::SetParametersResult result;
//      result.successful = true;
//      return result;
//
// Bonus (validation):
//   Check that kernel sizes are odd and positive.
//   If invalid, set result.successful = false and result.reason = "...".
// =============================================================================
rcl_interfaces::msg::SetParametersResult
EdgeDetectorNode::on_parameter_change(const std::vector<rclcpp::Parameter> & params)
{
  for (const auto& p : params) {
    if (p.get_name() == "algorithm") {
        algorithm_ = p.as_string();
    } else if (p.get_name() == "canny_low_threshold") {
        edge_params_.canny_low = p.as_int();
    }    else if (p.get_name() == "canny_high_threshold") {
        edge_params_.canny_high = p.as_int();
    }else if (p.get_name() == "sobel_kernel_size") {
        edge_params_.sobel_ksize = p.as_int();
    }else if (p.get_name() == "canny_low_threshold") {
        max_fps_ = p.as_double();
    }else if (p.get_name() == "target_width") {
        target_width_ = p.as_int();
    }
  }
  RCLCPP_INFO(this->get_logger(), "Parameters updated");
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace pi_edge_detector

// Register as a composable component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pi_edge_detector::EdgeDetectorNode)
