#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from go_to_service.action import DriveDistance  

class DriveDistanceServer(Node):
    def __init__(self):
        super().__init__('drive_distance_server')

        # pose tracking
        self.x0 = self.y0 = 0.0
        self.create_subscription(
            Odometry, '/wheel_odom_with_covariance',
            self.odom_cb, 10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._action_server = ActionServer(
            self,
            DriveDistance,
            'drive_distance',
            execute_callback=self.execute_cb
        )

    def odom_cb(self, msg):
        self.x0 = msg.pose.pose.position.x
        self.y0 = msg.pose.pose.position.y

    def execute_cb(self, goal_handle):
        goal = goal_handle.request
        goal_handle.accept_goal()

        target = goal.distance
        speed = goal.speed

        start_x, start_y = self.x0, self.y0

        feedback = DriveDistance.Feedback()
        self.get_logger().info(f"Driving {target:.2f} m @ {speed:.2f} m/s")

        rate = self.create_rate(10)  # 10 Hz
        travelled = 0.0
        while travelled < target and rclpy.ok():
            dx = self.x0 - start_x
            dy = self.y0 - start_y
            travelled = math.hypot(dx, dy)
            remaining = max(target - travelled, 0.0)

            feedback.remaining = remaining
            goal_handle.publish_feedback(feedback)

            cmd = Twist()
            cmd.linear.x = float(speed)
            self.cmd_pub.publish(cmd)

            rate.sleep()

        # stop
        self.cmd_pub.publish(Twist())

        result = DriveDistance.Result()
        result.success = True
        result.travelled = float(travelled)
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = DriveDistanceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
