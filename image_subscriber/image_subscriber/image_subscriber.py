#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Creates CvBridge connection
        self.bridge = CvBridge()

        # Subscribes to /camera/image_raw
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("Image Subscriber node initialized.")

    def image_callback(self, msg: Image):
        # Converts ROS Image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #Displays in Window
        cv2.imshow('Camera View', frame)
        # Refresh Window
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()