import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class ARFollowerNode(Node):
    def __init__(self):
        super().__init__('ar_follower_node')

        # Subscriber to camera images
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust if your topic is different
            self.image_callback,
            10
        )

        # Publisher to send velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # OpenCV bridge
        self.bridge = CvBridge()


        # 1) LOAD your marker image once
        share_dir = get_package_share_directory('ar_follower')
        template_path = os.path.join(share_dir, 'resource', 'marker0.png')
        self.template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
        if self.template is None:
            self.get_logger().error(f"Failed to load template at {template_path}")
            rclpy.shutdown()
            return

        # 2) SET UP ORB detector and BF matcher
        self.orb = cv2.ORB_create(1000)
        self.kp_template, self.des_template = self.orb.detectAndCompute(self.template, None)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Old way through dictionary 
        # ArUco dictionary (use standard 4x4_50 for compatibility)
        # self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_16h5) # different dictionary
        # self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        # self.parameters = cv2.aruco.DetectorParameters_create()

        # Control parameters
        self.target_distance = 0.8  # meters (desired distance from marker)
        self.linear_kp = 0.4  # proportional gain for linear velocity
        self.angular_kp = 1.0  # proportional gain for angular velocity

        # Camera parameters (assumed known/calibrated, replace with your values)
        self.camera_matrix = np.array([[600, 0, 320],
                                       [0, 600, 240],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5,1))

        self.get_logger().info("AR Follower node initialized!")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        kp_frame, des_frame = self.orb.detectAndCompute(gray, None)
        if des_frame is None:
            self.cmd_pub.publish(Twist())
            self.get_logger().warn("Marker not found, rover stopped.")
            return  # no features in frame

        matches = self.bf.match(self.des_template, des_frame)
        matches = sorted(matches, key=lambda x: x.distance)

        # c) Keep “good” matches
        good = matches[:50]  # tune this threshold
        if len(good) < 10:
            self.cmd_pub.publish(Twist())
            self.get_logger().warn(f"Too few matches ({len(good)}), stopping")
            return

        # d) Compute homography
        src_pts = np.float32([ self.kp_template[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp_frame [m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        if H is None:
            self.cmd_pub.publish(Twist())
            self.get_logger().warn("Homography failed, stopping")
            return

        # e) Project template corners to image to get polygon
        h, w = self.template.shape
        corners = np.float32([ [0,0],[w,0],[w,h],[0,h] ]).reshape(-1,1,2)
        projected = cv2.perspectiveTransform(corners, H)

        # f) Compute centroid & apparent width
        cx = projected[:,0,0].mean()
        pixel_width = np.linalg.norm(projected[1,0] - projected[0,0])

        # g) Convert to real‑world pose (using pinhole model)
        fx = self.camera_matrix[0,0]
        z = (0.10 * fx) / pixel_width          # 0.10 m = real marker width
        x = ((cx - self.camera_matrix[0,2]) / fx) * z

        # h) Proportional control
        linear_vel  = self.linear_kp * (z - self.target_distance)
        angular_vel = -self.angular_kp * x
        linear_vel  = np.clip(linear_vel, -0.2, 0.3)
        angular_vel = np.clip(angular_vel,-1.0, 1.0)

        cmd = Twist()
        cmd.linear.x  = float(linear_vel)
        cmd.angular.z = float(angular_vel)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f"Found marker: z={z:.2f} m, x={x:.2f} m")

        # # Detect markers
        # corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # if ids is not None and 0 in ids:
        #     idx = np.where(ids == 0)[0][0]  # Find marker ID 0
        #     marker_corners = corners[idx]

        #     # Estimate pose
        #     rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
        #         marker_corners, 0.10, self.camera_matrix, self.dist_coeffs)  # marker size = 10cm

        #     # Extract translation vector (marker position)
        #     x, y, z = tvec[0][0]

        #     # Control logic (simple proportional controller)
        #     linear_error = z - self.target_distance
        #     angular_error = x

        #     linear_vel = self.linear_kp * linear_error
        #     angular_vel = -self.angular_kp * angular_error

        #     # Cap velocities
        #     linear_vel = np.clip(linear_vel, -0.2, 0.3)
        #     angular_vel = np.clip(angular_vel, -1.0, 1.0)

        #     # Publish velocity
        #     cmd = Twist()
        #     cmd.linear.x = float(linear_vel)
        #     cmd.angular.z = float(angular_vel)
        #     self.cmd_pub.publish(cmd)

        #     self.get_logger().info(f"Marker detected! Distance: {z:.2f} m, Offset: {x:.2f} m")
        # else:
        #     # No marker detected; stop rover
        #     self.cmd_pub.publish(Twist())
        #     self.get_logger().info(f"  Debug → saw IDs: {ids.flatten() if ids is not None else '[]'}")

        #     # self.get_logger().warn("Marker not found, rover stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = ARFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
