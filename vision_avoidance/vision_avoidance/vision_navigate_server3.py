#!/usr/bin/env python3
import math, time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from go_to_interfaces.action import VisionNavigate


class VisionNavigateServer(Node):
    def __init__(self):
        super().__init__('vision_navigate_server')
        self.cb = ReentrantCallbackGroup()

        # ---- simple params ----
        self.declare_parameter('roi_y_ratio',       0.65)   # bottom 35% used for steering
        self.declare_parameter('front_band_ratio',  0.90)   # bottom 10% used for "blocked" check
        self.declare_parameter('floor_enter_th',    0.18)   # enter AVOID if front_floor < this
        self.declare_parameter('floor_exit_th',     0.28)   # leave AVOID if front_floor > this
        self.declare_parameter('h_tol',             12)     # hue tolerance (OpenCV hue units 0..179)
        self.declare_parameter('s_min',             35)     # min saturation to accept as floor
        self.declare_parameter('v_min',             35)     # min value (brightness)
        self.declare_parameter('kp_center',         1.2)    # steer toward floor centroid
        self.declare_parameter('kp_heading',        1.6)    # steer toward goal heading
        self.declare_parameter('max_yaw',           1.0)    # rad/s clamp
        self.declare_parameter('min_speed',         0.06)   # m/s crawl
        self.declare_parameter('search_yaw',        0.5)    # rad/s when blocked

        # IO
        self.bridge     = CvBridge()
        self.last_image = None
        self.odom       = None

        # learned floor model (very simple: one hue center + S/V gates)
        self.floor_h0   = None  # hue center in [0..179]
        self.floor_smin = None
        self.floor_vmin = None

        # mode: 'GOAL' or 'AVOID'
        self.mode = 'GOAL'

        # subs/pubs
        self.sub_img = self.create_subscription(Image, '/camera/image_raw',
                                                self.image_cb, 10, callback_group=self.cb)
        self.sub_odom = self.create_subscription(Odometry, '/wheel_odom',
                                                 self.odom_cb, 30, callback_group=self.cb)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.server = ActionServer(
            self, VisionNavigate, 'vision_navigate',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb
        )

        self.get_logger().info("Vision Navigate Server (simple floor follower) ready")

    # ---------- callbacks ----------
    def image_cb(self, msg: Image):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")

    def odom_cb(self, msg: Odometry):
        self.odom = msg

    def goal_cb(self, goal: VisionNavigate.Goal):
        self.get_logger().info(f"Goal: x={goal.target_x:.2f} y={goal.target_y:.2f} vmax={goal.max_speed:.2f}")
        return GoalResponse.ACCEPT

    def cancel_cb(self, _):
        self.get_logger().info("Goal cancelled")
        return CancelResponse.ACCEPT

    # ---------- helpers ----------
    def get_pose2d(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose
        x, y = float(p.position.x), float(p.position.y)
        q = p.orientation
        siny_cosp = 2.0*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return x, y, yaw

    def learn_floor_once(self, bgr):
        """Learn floor hue once from a small bottom-center patch."""
        h, w, _ = bgr.shape
        y0, y1 = int(0.88*h), int(0.96*h)
        x0, x1 = int(0.40*w), int(0.60*w)
        patch = bgr[y0:y1, x0:x1]
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        h0 = np.median(hsv[:,:,0].reshape(-1))
        self.floor_h0   = float(h0)
        self.floor_smin = float(self.get_parameter('s_min').value)
        self.floor_vmin = float(self.get_parameter('v_min').value)
        self.get_logger().info(f"Learned floor hue={self.floor_h0:.1f} Smin={self.floor_smin} Vmin={self.floor_vmin}")

    def floor_mask(self, bgr):
        """Binary mask where 255==floor, using single hue center + S/V gates."""
        if self.floor_h0 is None:
            self.learn_floor_once(bgr)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        H = hsv[:,:,0].astype(np.int16)
        S = hsv[:,:,1].astype(np.int16)
        V = hsv[:,:,2].astype(np.int16)

        h0   = int(self.floor_h0)
        htol = int(self.get_parameter('h_tol').value)

        # wrap-around hue distance
        dh = np.abs(H - h0)
        dh = np.minimum(dh, 180 - dh)

        mask = (dh <= htol) & (S >= self.floor_smin) & (V >= self.floor_vmin)
        mask = mask.astype(np.uint8) * 255

        # light morphology just to fill tiny gaps
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)
        return mask  # 0/255

    def scan_front(self, floor_mask):
        """
        Returns:
          front_floor: fraction of floor pixels in front band [0..1]
          cx_norm: x-centroid of floor in ROI, normalized to [-1..1] (None if no floor)
          side_bias: sign for which half has more floor (+right / -left)
        """
        h, w = floor_mask.shape

        # front band (blocked test)
        by = int(h * self.get_parameter('front_band_ratio').value)
        band = floor_mask[by:h, :]
        front_floor = float(band.mean()) / 255.0

        # ROI for steering (wider than the band)
        ry = int(h * self.get_parameter('roi_y_ratio').value)
        roi = floor_mask[ry:h, :]

        # centroid of floor in ROI
        ys, xs = np.nonzero(roi)
        cx_norm = None
        if xs.size > 0:
            cx = xs.mean()
            cx_norm = (cx / (w-1)) * 2.0 - 1.0  # [-1..1], 0 means centered

        # side bias for searching when blocked
        left = roi[:, :w//2].mean()
        right = roi[:, w//2:].mean()
        side_bias = 1.0 if right > left else -1.0

        return front_floor, cx_norm, side_bias

    # ---------- main action ----------
    async def execute_cb(self, gh):
        goal = gh.request
        vmax = max(0.0, goal.max_speed)
        kp_c = float(self.get_parameter('kp_center').value)
        kp_h = float(self.get_parameter('kp_heading').value)
        max_yaw = float(self.get_parameter('max_yaw').value)
        min_speed = float(self.get_parameter('min_speed').value)
        search_yaw = float(self.get_parameter('search_yaw').value)

        enter_th = float(self.get_parameter('floor_enter_th').value)
        exit_th  = float(self.get_parameter('floor_exit_th').value)

        t0 = self.get_clock().now().nanoseconds * 1e-9
        fb = VisionNavigate.Feedback()
        self.get_logger().info("Starting simple floor-follow navigation")

        while rclpy.ok():
            if gh.is_cancel_requested:
                self.cmd_pub.publish(Twist())
                gh.canceled()
                res = VisionNavigate.Result()
                res.success = False
                res.elapsed = float(self.get_clock().now().nanoseconds*1e-9 - t0)
                res.message = "Cancelled"
                return res

            pose = self.get_pose2d()
            if pose is None or self.last_image is None:
                time.sleep(0.05)
                continue

            x, y, yaw = pose

            # heading to goal (used in GOAL mode)
            dx = goal.target_x - x
            dy = goal.target_y - y
            remaining = math.hypot(dx, dy)
            desired_yaw = math.atan2(dy, dx)
            heading_err = (desired_yaw - yaw + math.pi) % (2*math.pi) - math.pi

            # floor sensing
            floor = self.floor_mask(self.last_image)
            front_floor, cx_norm, side_bias = self.scan_front(floor)

            # mode switch with hysteresis
            if self.mode == 'GOAL' and front_floor < enter_th:
                self.mode = 'AVOID'
            elif self.mode == 'AVOID' and front_floor > exit_th:
                self.mode = 'GOAL'

            cmd = Twist()

            if self.mode == 'AVOID':
                # turn toward side with more floor; creep a bit
                cmd.angular.z = float(side_bias * search_yaw) if cx_norm is None else float(kp_c * (-cx_norm))
                cmd.angular.z = max(-max_yaw, min(max_yaw, cmd.angular.z))
                cmd.linear.x  = min_speed if cx_norm is not None else 0.0
                state = f"AVOID front_floor={front_floor:.2f} cx={cx_norm if cx_norm is not None else 'None'}"
            else:
                # GOAL mode: blend goal heading + keep floor centered (small)
                ang = kp_h * heading_err
                if cx_norm is not None:
                    ang += 0.6 * kp_c * (-cx_norm)
                cmd.angular.z = max(-max_yaw, min(max_yaw, ang))

                # slow down when turning hard
                turn_scale = max(0.2, math.cos(abs(cmd.angular.z)))
                cmd.linear.x = max(min_speed, vmax * turn_scale)
                state = f"GOAL remain={remaining:.2f} cx={cx_norm if cx_norm is not None else 'None'}"

            # publish
            self.cmd_pub.publish(cmd)

            # feedback
            fb.remaining = float(remaining)
            fb.state = state
            gh.publish_feedback(fb)

            # success condition
            if remaining < 0.1 and self.mode == 'GOAL':
                self.cmd_pub.publish(Twist())
                gh.succeed()
                res = VisionNavigate.Result()
                res.success = True
                res.elapsed = float(self.get_clock().now().nanoseconds*1e-9 - t0)
                res.message = "Goal reached"
                return res

            time.sleep(0.05)  # ~20 Hz


def main(args=None):
    rclpy.init(args=args)
    node = VisionNavigateServer()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    ex.spin()
    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
