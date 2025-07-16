#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class ObjectFollower(Node):
    def __init__(self):
        super().__init__('object_follower')
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, 10)

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # simple color threshold (e.g. red object)
        mask = cv2.inRange(frame, (0,0,200), (50,50,255))
        M = cv2.moments(mask)
        twist = Twist()
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            error = cx - frame.shape[1]/2
            # proportional steering
            twist.angular.z = -float(error)/100.0
            # always move forward slowly
            twist.linear.x = 0.05
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = ObjectFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
