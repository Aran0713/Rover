#!/usr/bin/env python3
import math, rclpy, time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from go_to_interfaces.action import DriveDistance

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data




class DriveDistanceServer(Node):
    def __init__(self):
        super().__init__('drive_distance_server')
        
        self.cb_group = ReentrantCallbackGroup()

        # Subscriber to '/wheel_odom'
        self.sub = self.create_subscription(Odometry, 
            '/wheel_odom', 
            self.odom_cb, 
            qos_profile_sensor_data, 
            callback_group=self.cb_group
        )
        # Publisher to cmd
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Action Server
        self.action_server = ActionServer(
            self,
            DriveDistance,
            'drive_distance',
            execute_callback = self.execute_cb,
            goal_callback = self.goal_cb,
            cancel_callback = self.cancel_cb,
            callback_group=self.cb_group
        )

        # Variable for how far the rover has driven
        self.x = self.y = 0.0
        
        self.get_logger().info("Server Ready")
        

    
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.get_logger().info(f"Odom x: {self.x:.2f} , Odom y: {self.y:.2f} ")
        

    def goal_cb(self, goal_request: DriveDistance.Goal):
        self.get_logger().info(f"Goal received: distance = {goal_request.distance:.2f}, speed = {goal_request.speed:.2f}")
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    
    def execute_cb(self, goal_handle):
        # Target Distance
        target, speed = goal_handle.request.distance, goal_handle.request.speed
        self.get_logger().info(f"Target: {target} with speed {speed}")

        # Variables
        start_x, start_y = self.x, self.y
        feedback = DriveDistance.Feedback()
        travelled = 0.0


        try:
            while rclpy.ok() and travelled <= target:

                # Goal Canceled 
                if goal_handle.is_cancel_requested:
                    self.cmd_pub.publish(Twist())
                    goal_handle.canceled()
                    self.get_logger().info("Goal canceled")
                    result = DriveDistance.Result()
                    result.success = False
                    result.travelled = float(travelled)
                    return result


                dx = self.x - start_x
                dy = self.y - start_y
                travelled = math.hypot(dx, dy)
                remaining = max(target - travelled, 0.0)
                # self.get_logger().info(f"Odom x: {self.x:.2f} m, Odom y: {self.y:.2f} m")

                # Feedback
                feedback.remaining = remaining
                goal_handle.publish_feedback(feedback)

                # Drive the rover forward
                cmd = Twist()
                cmd.linear.x = float(speed)
                self.cmd_pub.publish(cmd)

                time.sleep(0.1)
                
            self.cmd_pub.publish(Twist()) # stop
            result = DriveDistance.Result()
            result.success = True
            result.travelled = float(travelled)
            goal_handle.succeed()
            self.get_logger().info(f"Travelled: {travelled:.2f} m")
            return result
        
        except Exception as e:
            self.get_logger().error(f"Exception occurred: {e}")
            result = DriveDistance.Result()
            result.success = False
            result.travelled = float(travelled)
            return result


def main(args=None):
    rclpy.init(args=args)
    node = DriveDistanceServer()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 

if __name__ == '__main__':
    main()