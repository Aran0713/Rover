import cv2
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class BlobFollower(Node):
    def __init__(self):
        super().__init__('blob_follower')

        # Parameters (color range and control gains)
        # HSV bounds min
        self.declare_parameter('h_min', 0)
        self.declare_parameter('s_min', 100)
        self.declare_parameter('v_min', 100)
        # HSV bounds max
        self.declare_parameter('h_max', 10)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_max', 255)
        # Ignore spurious specks
        self.declare_parameter('area_threshold', 500)
        # Gains for your P‑controller
        self.declare_parameter('linear_kp', 0.002)
        self.declare_parameter('angular_kp', 0.005)


        # Load them
        p = self.get_parameter
    
        self.lower = np.array([
            p('h_min').value,
            p('s_min').value,
            p('v_min').value,
        ], dtype=uint8)

        self.upper = np.array([
            p('h_max').value,
            p('s_max').value,
            p('v_max').value,
        ], dtype=uint8)

        self.area_thresh = p('area_threshold').value
        self.kp_lin = p('linear_kp').value
        self.kp_ang = p('angular_kp').value

        self.get_logger().info(f"Parameters set: Lower = {self.lower}, Upper = {self.upper}, area_thresh = {area_thresh}, kp_lin = {kp_lin}, kp_ang = {kp_ang}")


        # Subscriber
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/smd_vel', 10)

    
    def image_callback(self, msg: Image):

        # Convert image to HSV which is HSV color space where color segmentation
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor

        # Turns HSV space to binary mask so pixels turn white
        mask = cv2.inRange(hsv, self.lower, self.upper)
        # Cleans it up
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find all the white blobs
        cnts, _ = cv2.findContours(mask, cv2.ReTR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            self.cmd_pub.publish(Twist())
            return

        # Picking the largest contour
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < self.area_thresh:
            self.cmd_pub.pulish(Twist())
            return

        # Centroid
        M = cv2.moments(c)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        height, width = mask.shape
        err_x = cx - width/2
        linear_vel = float(self.kp_lin * area)
        angular_vel = float(-self.kp_ang * err_x)

        # Publish
        cmd = Twist()
        cmd.linear.x = max(min(linear_vel, 0.2), -0.2)
        cmd.angular.z = max(min(angular_vel, 1.0), -1.0)
        self.cmd_pub.publish(cmd)

        # Plot
        cv2.circle(frame, (cx, cy), 5, (0,255,0), -1)
        cv2.imshow('Blob Follower', frame)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = BlobFollower()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()