#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


class OdomDistanceNode(Node):

    def __init__(self):
        super().__init__('odom_distance')

        # Variables
        self.declare_parameter('odom_topic', '/wheel_odom')
        # self.declare_parameter('print_period', 1.0)
        # self.declare_parameter('min_dt', 0.0)

        # States
        odom_topic = self.get_parameter('odom_topic').value
        # self.print_period = float(self.get_parameter('print_period').value)
        # self.min_dt = float(self.get_parameter('min_dt').value)
        self.total_move = 0.0
        self.last_x = None
        self.last_y = None
        self.last_stamp = None
        # self.delta_since_last_print = 0.0

        # Subscriptions
        self.sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 50)
        # Service
        self.srv = self.create_service(Trigger, 'reset_distance', self.reset_cb)
        # Timer
        # self.timer = self.create_timer(self.print_period, self.print_cb)

        # Log
        self.get_logger().info("Odom is listening, call /reset_distance to reset counter")


    def odom_cb(self, msg: Odometry):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        stamp = Time.from_msg(msg.header.stamp)

        # First run
        if self.last_x is None:
            self.last_x, self.last_y, self.last_stamp = x, y, stamp
            return

        dx = x - self.last_x
        dy = y - self.last_y
        ds = math.hypot(dx, dy)
        self.total_move += ds

        # reset variables
        self.last_x, self.last_y, self.last_stamp = x, y, stamp

        if ds == 0.0:
            return

        # Log
        self.get_logger().info(
            f"Distance Total = {self.total_move:.3f} m"
            f"Distance Moved right now= {ds:.3f} m"
        )

    def reset_cb(self, request, response):
        self.total_move = 0.0
        response.success = True
        response.message = "Distance counter reset"
        self.get_logger().info(response.message)
        return response



def main(args=None):
    rclpy.init(args=args)
    node = OdomDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()