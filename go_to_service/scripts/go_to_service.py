#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from go_to_service.srv import GoToDistance
from go_to_service.action import DriveDistance
from rclpy.action import ActionClient

class GoToServiceNode(Node):
    def __init__(self):
        super().__init__('go_to_service_node')
        self._action_client = ActionClient(self, DriveDistance, 'drive_distance')
        self._srv = self.create_service(
            GoToDistance,
            'go_to',
            self.handle_go_to
        )
        self.get_logger().info("Service /go_to ready: distance+speed -> drives for you")

    def handle_go_to(self, request, response):
        dist = request.distance
        spd = request.speed

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            response.success = False
            response.message = "Action server unavailable"
            return response

        goal_msg = DriveDistance.Goal()
        goal_msg.distance = dist
        goal_msg.speed = spd

        send_goal = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal)
        goal_handle = send_goal.result()

        if not goal_handle.accepted:
            response.success = False
            response.message = "Goal rejected"
            return response

        get_result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result)
        result = get_result.result().result

        response.success = result.success
        response.message = f"Travelled {result.travelled:.2f} m"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GoToServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
