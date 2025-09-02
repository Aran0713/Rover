#!/usr/bin/env python3
from statistics import mean
import math, time
import numpy as np
import cv2
# import asyncio  

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
# from rclpy.rate import Rate
# from rclpy import rate

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from go_to_interfaces.action import VisionNavigate

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class VisionNavigateServer(Node):
    def __init__(self):
        super().__init__('vision_navigate_server')
        
        self.cb_group = ReentrantCallbackGroup()  
        
        self.declare_parameter('roi_y_ratio', 0.60) # Analyzing the bottom 40% of the screen - 60
        self.declare_parameter('front_band_ratio', 0.85) # Emergency band near bottom - 85
        self.declare_parameter('obs_thresh', 0.70) # Stop if obstacle is taking up 35% of the screen
        
        self.declare_parameter('kp_heading', 1.8) # How strongly to turn towards the goal
        self.declare_parameter('kp_avoid', 1.2) # How strongly to turn away from obstacles
        self.declare_parameter('max_yaw', 1.2) # Limit how fast the rover can turn
        self.declare_parameter('min_speed', 0.05)  

        # Subscribers, Publishers, Server
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10, callback_group=self.cb_group)
        self.sub_odom = self.create_subscription(Odometry, '/wheel_odom', self.odom_callback, 50, callback_group=self.cb_group)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.server = ActionServer(
            self,
            VisionNavigate,
            'vision_navigate',
            execute_callback = self.execute_callback,
            goal_callback = self.goal_callback,
            cancel_callback = self.cancel_callback,
            callback_group=self.cb_group
        )
        
        # I/O, working buffers
        self.bridge = CvBridge()
        self.last_image = None
        self.odom = None
        self.floor_hsv = None
        
        self.get_logger().info("Vision Navigate Server is ready")
        
        
    def image_callback(self, msg: Image):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"CV Bridge error: {e}")
            
    
    def odom_callback(self, msg: Odometry):
        self.odom = msg
        self.get_logger().info(f"Received odometry: {msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}")



    def goal_callback(self, goal: VisionNavigate.Goal):
        self.get_logger().info(f"Received goal target: x = {goal.target_x:.2f}, y = {goal.target_y:.2f}, vmax={goal.max_speed:.2f}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, _):
        self.get_logger().info("Goal cancelled")
        return CancelResponse.ACCEPT
    
    def get_pose2d(self):
        if self.odom is None:
            return None
        
        pos = self.odom.pose.pose
        x, y = float(pos.position.x), float(pos.position.y)
        q = pos.orientation
        # From a quaternion to the Z-YX Euler yaw (rotation-matrix formulas)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return x, y, yaw
    
    def learn_floor(self, bgr):
        h, w, _ = bgr.shape
        y0 = int(h*0.90); y1 = int(h*0.98)
        x0 = int(w*0.35); x1 = int(w*0.65)
        patch = bgr[y0:y1, x0:x1]

        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        mean = hsv.reshape(-1, 3).mean(axis=0)
        std  = hsv.reshape(-1, 3).std(axis=0)

        lo = np.clip(mean - 2.5*std - np.array([5, 40, 40]), [0,0,0], [179,255,255])
        hi = np.clip(mean + 2.5*std + np.array([5, 40, 40]), [0,0,0], [179,255,255])
        self.floor_hsv = (lo.astype(np.uint8), hi.astype(np.uint8))
    
    def obstacle_mask(self, bgr):
        if self.floor_hsv is None:
            self.learn_floor(bgr)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lo, hi = self.floor_hsv
        floor = cv2.inRange(hsv, lo, hi)
        obs = cv2.bitwise_not(floor)

        h, w = obs.shape
        roi_y = int(h * self.get_parameter('roi_y_ratio').value)
        roi = obs[roi_y:h, :]

        kernel = np.ones((5,5), np.uint8)
        roi = cv2.morphologyEx(roi, cv2.MORPH_OPEN, kernel, iterations=1)
        roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel, iterations=1)

        col_sum = roi.sum(axis=0).astype(np.float32)
        col_sum /= 255.0 * (h - roi_y)
        free = 1.0 - col_sum

        w_window = max(7, int(w/30))
        free_smooth = cv2.blur(free, (w_window, 1)).flatten()
        best_col = int(np.argmax(free_smooth))
        steer_norm = (best_col / (w-1))*2.0 - 1.0

        band_y = int(h * self.get_parameter('front_band_ratio').value)
        band = obs[band_y:h, :]
        front_fill = band.mean() / 255.0

        return obs, steer_norm, front_fill
        

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        # period_s = 1.0 / 20.0
        t0 = self.get_clock().now().nanoseconds * 1e-9

        feedback = VisionNavigate.Feedback()
        self.get_logger().info("Starting vision navigation")

        try:
            while rclpy.ok():
                # Cancellation
                if goal_handle.is_cancel_requested:
                    self.cmd_pub.publish(Twist())
                    goal_handle.canceled()
                    result = VisionNavigate.Result()
                    result.success = False
                    result.elapsed = float(self.get_clock().now().nanoseconds * 1e-9 - t0)
                    result.message = "Goal cancelled"
                    return result

                pose = self.get_pose2d()
                if pose is None or self.last_image is None:
                    self.get_logger().warn("Waiting for odom/image")
                    continue

                x, y, yaw = pose

                # Navigation geometry
                dx = goal.target_x - x
                dy = goal.target_y - y
                remaining = math.hypot(dx, dy)
                desired_yaw = math.atan2(dy, dx)
                heading_err = (desired_yaw - yaw + math.pi) % (2 * math.pi) - math.pi

                # Vision avoidance
                _, steer_bias, front_fill = self.obstacle_mask(self.last_image)

                # Control
                kp_h   = self.get_parameter('kp_heading').value
                kp_a   = self.get_parameter('kp_avoid').value
                max_yaw = self.get_parameter('max_yaw').value

                ang = kp_h * heading_err - kp_a * steer_bias
                ang = max(-max_yaw, min(max_yaw, ang))

                vmax = max(0.0, goal.max_speed)
                speed_scale = max(0.0, 1.0 - front_fill) * max(0.2, math.cos(abs(ang)))
                v = vmax * speed_scale
                if v > 0.0:
                    v = max(v, self.get_parameter('min_speed').value)

                if front_fill >= self.get_parameter('obs_thresh').value:
                    v = 0.0
                    state = f"blocked ({front_fill:.2f})"
                else:
                    state = f"steer = {steer_bias:.2f}, heading error = {heading_err:.2f}"

                # Publish cmd
                cmd = Twist()
                cmd.linear.x  = float(v)
                cmd.angular.z = float(ang)
                self.cmd_pub.publish(cmd)

                # Feedback
                feedback.remaining = float(remaining)
                feedback.state = state
                goal_handle.publish_feedback(feedback)

                # Completion
                if remaining < 0.1 and abs(heading_err) < 0.25:
                    self.cmd_pub.publish(Twist())
                    goal_handle.succeed()
                    result = VisionNavigate.Result()
                    result.success = True
                    result.elapsed = float(self.get_clock().now().nanoseconds * 1e-9 - t0)
                    result.message = "Goal reached"
                    return result


        except Exception as e:
            self.get_logger().exception(f"Execute exception: {e}")
            self.cmd_pub.publish(Twist())
            goal_handle.abort()
            result = VisionNavigate.Result()
            result.success = False
            result.elapsed = float(self.get_clock().now().nanoseconds * 1e-9 - t0)
            result.message = f"Exception: {e}"
            return result




def main(args=None):
    rclpy.init(args=args)
    node = VisionNavigateServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    # rclpy.spin(node)

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
