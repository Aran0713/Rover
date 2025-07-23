#!/usr/bin/env python3
import sys, select, termios, tty
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist



class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        # Create publisher on /cmd_vel
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.settings = termios.tcgetattr(sys.stdin)
        # Log on Terminal 
        self.get_logger().info("Teleop node intialized. Press W,A,S,D to drive and Q to quit")

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


    def run(self):

        try:

            # Loop until ROS is shutdown
            while rclpy.ok():
                key = self.get_key()

                if key == 'q':
                    break

                # Build Twist
                twist = Twist()
                
                if key == 'w':
                    twist.linear.x = 10.0
                elif key == 's':
                    twist.linear.x = -10.0
                elif key == 'a':
                    twist.angular.z = 10.0
                elif key == 'd':
                    twist.angular.z = -10.0
                # else:
                #     twist.linear.x = 0.0
                #     twist.angular.z = 0.0

                # Sends the command to the topic /cmd_vel and logs it
                self.pub.publish(twist)
                if key:
                    self.get_logger().info(f"Key '{key}': lin{twist.linear.x}, ang={twist.angular.z}")
                # self.get_logger().info(f"Key '{key}': lin{twist.linear.x}, ang={twist.angular.z}")

        except Exception as e:
            self.get_logger().error(f"Error in teleop loop: {e}")

        finally:

            # Sends topic a 0 velocity
            stop = Twist()
            self.pub.publish(stop)
            # Returns terminal to old way 
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("Teleop node shutting down")

                

def main(args=None):

    # Boots up Ros 2
    rclpy.init(args=args) 
    # Creates an instance of the TeleopNode
    node = TeleopNode()
    # Calls the loop function
    node.run()

    # Clean up
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()





