#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from go_to_interfaces.srv import GoToDistance
from go_to_interfaces.action import DriveDistance
from rclpy.action import ActionClient

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

class GoToServiceNode(Node):
    def __init__(self):
        super().__init__('go_to_service_node')
        
        # allow callbacks to run concurrently
        self.cb_group = ReentrantCallbackGroup()
        
        self.action_client = ActionClient(self, DriveDistance, 'drive_distance', callback_group=self.cb_group)
        self._srv = self.create_service(
            GoToDistance,
            'go_to',
            self.handle_go_to,
            callback_group=self.cb_group
        )
        
        self.get_logger().info("Client Ready")

    def handle_go_to(self, req, res):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            res.success = False
            res.message = "Action server unavailable"
            return res

        goal = DriveDistance.Goal()
        goal.distance, goal.speed = req.distance, req.speed
        

        # send_goal = self.action_client.send_goal_async(goal)
        # rclpy.spin_until_future_complete(self, send_goal)
        # result = send_goal.result()

        # if not result.accepted:
        #     res.success = False
        #     res.message = "Goal rejected"
        #     return res

        # get_res = result.get_result_async()
        # rclpy.spin_until_future_complete(self, get_res)

        # result_msg = get_res.result().result
        # res.success = result_msg.success
        # res.message = f"Travelled {result_msg.travelled:.2f} m"
        # return res
        


        # Threading multiple callbacks
        done = threading.Event()
        out = {"ok": False, "msg": "Unknown", "travelled": 0.0}

        def on_result(fut):
            try:
                r = fut.result().result           
                out["ok"] = bool(r.success)
                out["travelled"] = float(r.travelled)
                out["msg"] = f"Travelled {r.travelled:.2f} m"
            except Exception as e:
                out["ok"] = False
                out["msg"] = f"Action result error: {e}"
            finally:
                done.set()

        def on_goal_sent(fut):
            try:
                gh = fut.result()                 
                if not gh.accepted:
                    out["ok"] = False
                    out["msg"] = "Goal rejected"
                    done.set()
                    return
                gh.get_result_async().add_done_callback(on_result)
            except Exception as e:
                out["ok"] = False
                out["msg"] = f"Goal send error: {e}" 
                done.set()

        # Sending the goal
        self.action_client.send_goal_async(goal).add_done_callback(on_goal_sent)

        if not done.wait(timeout=120.0):
            res.success = False
            res.message = "Timed out waiting for action result"
            return res

        res.success = out["ok"]
        res.message = out["msg"]
        return res
        



def main(args=None):
    rclpy.init(args=args)
    node = GoToServiceNode()
    
    # multi-threaded
    executor = MultiThreadedExecutor()   
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
