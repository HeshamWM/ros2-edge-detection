#ifndef PI_EDGE_DETECTOR__EDGE_DETECTOR_NODE_HPP_
#define PI_EDGE_DETECTOR__EDGE_DETECTOR_NODE_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "pi_edge_detector/edge_algorithms.hpp"
#include "pi_edge_detector/perf_monitor.hpp"

namespace pi_edge_detector {

/// Main ROS 2 node that orchestrates the edge detection pipeline.
///
/// Lifecycle:
///   1. Constructor declares parameters, creates pub/sub/timer
///   2. image_callback() fires on every incoming frame
///   3. diagnostics_callback() fires at 1 Hz to publish perf stats
///
/// Threading:
///   Designed to run inside component_container_mt. The executor will
///   dispatch image_callback and diagnostics_callback on separate threads.
///   No shared mutable state is accessed from both without care:
///     - cv::Mat buffers are ONLY touched in image_callback
///     - PerfMonitor is touched from both → you'll need to think about this
///
/// YOUR TASKS (see the .cpp file):
///   Task 1: Declare and read all ROS 2 parameters in the constructor
///   Task 2: Set up image_transport subscriber and publisher
///   Task 3: Implement the image_callback processing pipeline
///   Task 4: Wire up dynamic parameter reconfiguration
///   Task 5: Implement diagnostics publishing
class EdgeDetectorNode : public rclcpp::Node {
public:
  explicit EdgeDetectorNode(const rclcpp::NodeOptions & options);

private:
  // ---- Callbacks (YOU implement these) ----

  /// Called on every incoming frame from image_transport.
  /// This is where the main processing pipeline lives.
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /// Called at 1 Hz by a wall timer to publish diagnostics.
  void diagnostics_callback();

  /// Called when any parameter is changed at runtime.
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> & params);

  // ---- ROS interfaces ----
  image_transport::Subscriber image_sub_;
  image_transport::Publisher  image_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ---- Configuration (from ROS parameters) ----
  std::string algorithm_;
  EdgeParams  edge_params_;
  int         target_width_;
  double      max_fps_;
  bool        publish_diagnostics_;

  // ---- Pre-allocated processing buffers ----
  // Reusing these across frames avoids heap allocation per frame.
  cv::Mat resize_buffer_;
  cv::Mat gray_buffer_;
  cv::Mat edge_buffer_;

  // ---- Performance tracking ----
  PerfMonitor perf_monitor_;
  rclcpp::Time last_process_time_;
};

}  // namespace pi_edge_detector

#endif  // PI_EDGE_DETECTOR__EDGE_DETECTOR_NODE_HPP_
