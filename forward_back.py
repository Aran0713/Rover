import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def main():
    rclpy.init()
    node = Node('forward_back')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    # 2 cm at 0.02 m/s for 1 s → 10 publishes at 10 Hz
    twist = Twist()
    twist.linear.x = 0.05
    for _ in range(10):
        pub.publish(twist)
        time.sleep(0.1)

    # stop
    twist.linear.x = 0.0
    pub.publish(twist)
    time.sleep(0.1)

    # back 2 cm
    twist.linear.x = -0.05
    for _ in range(10):
        pub.publish(twist)
        time.sleep(0.1)

    # stop
    twist.linear.x = 0.0
    pub.publish(twist)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
