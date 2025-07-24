#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist



class ToggleServiceNode(Node):
    def __init__(self):
        super().__init__('toggle_service_node')

        # Toggle
        self.vision_enabled = False

        # Sub, Pub
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            1
        )

        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # Service 
        self.srv = self.create_service(
            SetBool,
            'enable_vision',
            self.enable_callback
        )

        self.get_logger().info("Toggle is ready (call /enable_vision to turn on and off):")


    def enable_callback(self, request, response):
        # Toggle will be true or false
        self.vision_enabled = request.data

        response.success = True
        response.message = "Vision " + ("enabled" if request.data else "disabled")
        self.get_logger().info(response.message)


        if not request.data:
            self.pub.publish(Twist()) # stops the rover

    def image_callback(self, msg):
        
        if not self.vision_enabled:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]
        cv2.rectangle(frame, (0,0), (w-1,h-1), (0,255,0), 5)
        cv2.imshow('Vision Enabled', frame)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = ToggleServiceNode()
    rclpy.spin(node)

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()