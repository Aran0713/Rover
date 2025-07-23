#!/usr/bin/env python3
import cv2, numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist



class OpticalStopper(Node):
    def __init__(self):
        super().__init__('optical_stopper')
        self.bridge = CvBridge()
        # Intialize the previous image as None so that the function will run twice before the check
        self.prev_gray = None

        # Sensitivity
        self.declare_parameter('flow_threshold', 2.0)
        self.flow_thresh = self.get_parameter('flow_threshold').value

        # Subscription to raw image
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.cb_image, 5
        )

        # Publisher to cmd_vel
        self.pub = self.create_publisher(Twist, "/cmd_vel", 5)

        self.get_logger().info("Intialized Stopper")

    def cb_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # To have two images to compare
        if self.prev_gray is None:
            self.prev_gray = gray
            return

        # Compute Farneback flow
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray, gray, None, 
            0.5, 3, 15, 3, 5, 1.2, 0
        )
        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        mean_flow = np.mean(mag)

        # If rover is too close, it will stop
        if mean_flow > self.flow_thresh:
            self.get_logger().warn("Obstacle nearby - Stopping Rover")
            self.pub.publish(Twist()) # Note: Twist() means 0 velocity


        # Log mean flow
        self.get_logger().info(f"Mean flow: {mean_flow:.2f}")
        # Compare to next image
        self.prev_gray = gray




def main(args=None):
    rclpy.init(args=args)
    node = OpticalStopper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()