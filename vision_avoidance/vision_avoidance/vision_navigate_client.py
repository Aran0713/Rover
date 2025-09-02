#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from go_to_interfaces.action import VisionNavigate

class Client(Node):
    def __init__(self):
        super().__init__('vision_nav_client')
        self.ac = ActionClient(self, VisionNavigate, 'vision_navigate')

    def send(self, x, y, vmax=0.25):
        if not self.ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Server not available")
            return
        goal = VisionNavigate.Goal()
        goal.target_x, goal.target_y, goal.max_speed = float(x), float(y), float(vmax)
        fut = self.ac.send_goal_async(goal, feedback_callback=self.fb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        self.get_logger().info(f"Result: {res_fut.result().result}")

    def fb(self, msg):
        fb = msg.feedback
        self.get_logger().info(f"remaining={fb.remaining:.2f} state={fb.state}")

def main():
    rclpy.init()
    node = Client()
    node.send(0.3, 0.0, 0.1)   # drive 1 m 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
