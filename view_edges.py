#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class EdgeViewer(Node):
    def __init__(self):
        super().__init__('edge_viewer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/edge_detector/edges',
            self.listener_callback,
            10
        )
        self.get_logger().info('Viewer started. Waiting for images on /edge_detector/edges...')

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV Mat
            # We use "mono8" because our edge detector publishes single-channel images
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            
            # Display image in a window
            cv2.imshow("Processed Edges", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    viewer = EdgeViewer()
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
