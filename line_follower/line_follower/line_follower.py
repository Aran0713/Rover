#!/usr/bin/env python3
import cv2
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        # Parameters
        self.declare_parameter('threshold', 150) # Gray threshold
        self.declare_parameter('roi_y', 200) # Vertical region to scan
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_kp', 0.005)
        self.declare_parameter('timer_hz', 10.0) # Timer calls

        p = self.get_parameter
        self.thresh     = p('threshold').value
        self.roi_y      = p('roi_y').value
        self.lin_speed  = p('linear_speed').value
        self.kp_ang     = p('angular_kp').value
        hz              = p('timer_hz').value

        self.get_logger().info(
            f"[LineFollower] gray<{self.thresh}, roi_y={self.roi_y}, "
            f"lin={self.lin_speed}, kp_ang={self.kp_ang}, hz={hz}"
        )

        # Pub and sub
        self.bridge = CvBridge()
        self.sub    = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 1)
        self.pub    = self.create_publisher(Twist, '/cmd_vel', 1)

        # Latest frame
        self.latest_frame = None
        # Timer calls timer_cb at hz per second so hz = 10, it'll be 10 times per second
        self.timer = self.create_timer(1.0/hz, self.timer_cb)

    # Stores latest frame
    def image_cb(self, msg: Image):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def timer_cb(self):

        if self.latest_frame is None:
            return

        frame = self.latest_frame
        h, w = frame.shape[:2]

        # Crop bottom of the frame
        roi = frame[self.roi_y:h, 0:w]

        # Turning line to a white blob
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, bw = cv2.threshold(gray, self.thresh, 255, cv2.THRESH_BINARY_INV)

        # Finding contour
        cnts, _ = cv2.findContours(
            bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            self.pub.publish(Twist())
            return
        c = max(cnts, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] == 0:
            self.pub.publish(Twist())
            return
        cx = int(M['m10']/M['m00'])
        err_x = cx - (w/2)

        # Publishing 
        cmd = Twist()
        cmd.linear.x  = float(self.lin_speed)
        cmd.angular.z = float(-self.kp_ang * err_x)
        self.pub.publish(cmd)

        # Visualization
        cv2.circle(roi, (cx, int((h-self.roi_y)/2)), 5, (0,255,0), -1)
        cv2.imshow('LineFollower', roi)
        cv2.waitKey(1)




def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()