#ifndef PI_YOLOV26_DETECTOR__YOLOV26_DETECTOR_NODE_HPP_
#define PI_YOLOV26_DETECTOR__YOLOV26_DETECTOR_NODE_HPP_

#include <string>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include "pi_yolov26_detector/perf_monitor.hpp"

namespace pi_yolov26_detector {

class Yolov26DetectorNode : public rclcpp::Node {
public:
  explicit Yolov26DetectorNode(const rclcpp::NodeOptions & options);

private:
  /// Callback triggered on every incoming image frame.
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  /// Periodic callback for publishing performance metrics.
  void diagnostics_callback();

  /// Handle dynamic parameter reconfiguration.
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> & params);

  /// Load the YOLOv26 ONNX model using OpenCV DNN.
  void load_model();

  // ---- ROS interfaces ----
  image_transport::Subscriber image_sub_;
  image_transport::Publisher  image_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ---- Configuration Parameters ----
  std::string model_path_;
  double      confidence_threshold_;
  int         target_width_;
  double      max_fps_;
  bool        publish_diagnostics_;

  // ---- OpenCV DNN net ----
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;

  // ---- Pre-allocated processing buffers ----
  cv::Mat resize_buffer_;
  cv::Mat blob_buffer_;
  cv::Mat output_buffer_;

  // ---- Performance tracking ----
  PerfMonitor perf_monitor_;
  rclcpp::Time last_process_time_;
};

}  // namespace pi_yolov26_detector

#endif  // PI_YOLOV26_DETECTOR__YOLOV26_DETECTOR_NODE_HPP_
