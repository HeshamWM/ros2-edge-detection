#ifndef MAC_CAMERA_PUBLISHER__CAMERA_PUBLISHER_NODE_HPP_
#define MAC_CAMERA_PUBLISHER__CAMERA_PUBLISHER_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

namespace mac_camera_publisher {

class CameraPublisherNode : public rclcpp::Node {
public:
  explicit CameraPublisherNode(const rclcpp::NodeOptions & options);
  ~CameraPublisherNode() override;

private:
  void timer_callback();

  // ROS interfaces
  image_transport::Publisher image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  int camera_index_;
  double fps_;
  std::string frame_id_;

  // OpenCV camera capture
  cv::VideoCapture cap_;
  cv::Mat frame_;
  uint64_t frame_count_;
};

}  // namespace mac_camera_publisher

#endif  // MAC_CAMERA_PUBLISHER__CAMERA_PUBLISHER_NODE_HPP_
