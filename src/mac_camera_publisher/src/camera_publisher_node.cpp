#include "mac_camera_publisher/camera_publisher_node.hpp"
#include <cv_bridge/cv_bridge.h>

namespace mac_camera_publisher {

CameraPublisherNode::CameraPublisherNode(const rclcpp::NodeOptions & options)
  : Node("camera_publisher", options),
    frame_count_(0)
{
  // 1. Declare and retrieve parameters
  this->declare_parameter<int>("camera_index", 0);
  this->declare_parameter<double>("fps", 30.0);
  this->declare_parameter<std::string>("frame_id", "camera_frame");

  camera_index_ = this->get_parameter("camera_index").as_int();
  fps_ = this->get_parameter("fps").as_double();
  frame_id_ = this->get_parameter("frame_id").as_string();

  RCLCPP_INFO(this->get_logger(), "Initializing camera index %d at %.1f FPS", camera_index_, fps_);

  // 2. Initialize camera
  cap_.open(camera_index_);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera index %d!", camera_index_);
    throw std::runtime_error("Failed to open camera device.");
  }

  // 3. Set up publisher
  image_pub_ = image_transport::create_publisher(this, "camera/image_raw");

  // 4. Set up capture and publish timer
  int period_ms = static_cast<int>(1000.0 / fps_);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&CameraPublisherNode::timer_callback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Camera publisher started successfully!");
}

CameraPublisherNode::~CameraPublisherNode()
{
  if (cap_.isOpened()) {
    cap_.release();
  }
  RCLCPP_INFO(this->get_logger(), "Camera capture released.");
}

void CameraPublisherNode::timer_callback()
{
  if (!cap_.read(frame_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to grab frame!");
    return;
  }

  if (frame_.empty()) {
    RCLCPP_WARN(this->get_logger(), "Grabbed empty frame!");
    return;
  }

  // Convert OpenCV Mat to ROS 2 Image message
  std::string encoding = "bgr8"; // Mac webcam output is 3-channel color
  auto msg = cv_bridge::CvImage(
    std_msgs::msg::Header(),
    encoding,
    frame_
  ).toImageMsg();

  // Populate header stamp and frame ID
  msg->header.stamp = this->now();
  msg->header.frame_id = frame_id_;

  // Publish
  image_pub_.publish(*msg);
  frame_count_++;
}

}  // namespace mac_camera_publisher

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mac_camera_publisher::CameraPublisherNode)
