#!/usr/bin/env python3
import sys, select, termios, tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Text to show the user
HELP_TEXT = """
Keyboard Teleop Node
--------------------
Use W/S to move forward/back,
    A/D to turn left/right,
    Q to quit.
"""

# Key → (linear_vel, angular_vel) mappings
MOVE_BINDINGS = {
    'w': ( 0.2,  0.0),
    's': (-0.2,  0.0),
    'a': ( 0.0,  0.5),
    'd': ( 0.0, -0.5),
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        # 1) Create a Publisher on /cmd_vel for Twist messages
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 2) Save terminal settings so we can restore later
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info("Teleop node initialized. Press keys to drive.")

    def get_key(self, timeout=0.1):
        # Put terminal into raw mode to read single characters
        tty.setraw(sys.stdin.fileno())
        # Wait 'timeout' seconds for input
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(HELP_TEXT)
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'q':
                    break

                twist = Twist()
                if key in MOVE_BINDINGS:
                    lin, ang = MOVE_BINDINGS[key]
                    twist.linear.x  = lin
                    twist.angular.z = ang
                    self.get_logger().info(f"Key '{key}': lin={lin}, ang={ang}")
                else:
                    # on any other key, zero velocities
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                # Send the command
                self.pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Error in teleop loop: {e}")

        finally:
            # On exit, stop the robot
            stop = Twist()
            self.pub.publish(stop)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("Teleop node shutting down.")

def main(args=None):
    # 1) Initialize ROS2 Python context
    rclpy.init(args=args)

    # 2) Create our node
    node = TeleopNode()

    # 3) Spin up keyboard loop
    node.run()

    # 4) Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
