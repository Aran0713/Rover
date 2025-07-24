#!/usr/bin/env python3

import sys, select, termios, tty
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist


class ToggleTeleopNode(Node):
    def __init__(self):
        super().__init__('toggle_teleop_node')

        # Toggle
        self.toggle = False

        # Pub
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)

        self.settings = termios.tcgetattr(sys.stdin)

        # Service
        self.srv = self.create_service(
            SetBool,
            'enable_teleop',
            self.enable_callback
        )

        # Timer
        self.timer = self.create_timer(0.1, run_teleop)

        self.get_logger().info("Toggle is ready (call /enable_teleop to turn on and off) and Press W,A,S,D to drive")



    def enable_callback(self, request, response):
        # Toggle will be true or false
        self.toggle = request.data

        response.success = True
        response.message = "Teleop " + ("enabled" if request.data else "disabled")
        self.get_logger().info(response.message)

        if not request.data:
            self.pub.publish(Twist())

        return response

    
    def get_key(self, timeout=0.1):

        # Switch terminal to raw mode
        tty.setraw(sys.stdin.fileno())

        # Wait for keypress
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        # Revert terminal to old settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def run_teleop(self):

        if not self.toggle:
            return

        try:

            # Loop if service gives true
            key = self.get_key()

            # Build Twist
            twist = Twist()
            if key == 'w':
                twist.linear.x = 1.0
            elif key == 's':
                twist.linear.x = -1.0
            elif key == 'a':
                twist.angular.z = 1.0
            elif key == 'd':
                twist.angular.z = -1.0

            # Sends the command to the topic /cmd_vel and logs it
            self.pub.publish(twist)
            if key:
                self.get_logger().info(f"Key '{key}': lin{twist.linear.x}, ang={twist.angular.z}")

        except Exception as e:
            self.get_logger().error(f"Error in teleop loop: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ToggleTeleopNode()
    rclpy.spin(node)

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()