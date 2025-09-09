#!/usr/bin/env python3
import json, math, socket, struct, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# YAMCS_PARAM_HOST = '127.0.0.1'
YAMCS_PARAM_HOST = '10.0.0.186'
YAMCS_PARAM_PORT = 10050      # udp-params-in
YAMCS_TC_LISTEN_PORT = 10051  # udp-tc-out (we listen here)

class YamcsRosBridge(Node):
    def __init__(self):
        super().__init__('yamcs_ros_bridge')
        self.sub = self.create_subscription(Odometry, '/wheel_odom', self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # UDP sockets
        self.param_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tc_sock.bind(('0.0.0.0', YAMCS_TC_LISTEN_PORT))
        self.tc_sock.setblocking(False)

        # distance 
        self._prev_xy = None
        self._dist_total = 0.0

        # timer to poll for commands 
        self.create_timer(0.02, self.poll_tc)

        # Commands control states
        self.active = None     
        self.queue  = []       
        self._max_lin = 10.0 #    
        self._max_ang = 10.0  # 1.0
        self._last_yaw = 0.0

        self.get_logger().info("Bridge Established")


    def odom_cb(self, msg: Odometry):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        # Yaw from quaternion 
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self._last_yaw = float(yaw)

        lin = msg.twist.twist.linear.x
        ang = msg.twist.twist.angular.z

        # integrate distance in 2D
        if self._prev_xy is not None:
            dx = x - self._prev_xy[0]
            dy = y - self._prev_xy[1]
            self._dist_total += math.hypot(dx, dy)
        self._prev_xy = (x, y)

        # self.get_logger().info(f"X: {x}")

        payload = {
            "parameter": [
                {"id": {"name": "/leorover/ODOM/x"},               "engValue": {"type":"DOUBLE","doubleValue": float(x)}},
                {"id": {"name": "/leorover/ODOM/y"},               "engValue": {"type":"DOUBLE","doubleValue": float(y)}},
                {"id": {"name": "/leorover/ODOM/yaw"},             "engValue": {"type":"DOUBLE","doubleValue": float(yaw)}},
                {"id": {"name": "/leorover/ODOM/linear_speed"},    "engValue": {"type":"DOUBLE","doubleValue": float(lin)}},
                {"id": {"name": "/leorover/ODOM/angular_speed"},   "engValue": {"type":"DOUBLE","doubleValue": float(ang)}},
                {"id": {"name": "/leorover/ODOM/distance_total"},  "engValue": {"type":"DOUBLE","doubleValue": float(self._dist_total)}},
            ]
        }
        data = json.dumps(payload).encode('utf-8')
        self.param_sock.sendto(data, (YAMCS_PARAM_HOST, YAMCS_PARAM_PORT))
        # self.get_logger().info(f"Data: {data}")


    def _enqueue_or_start(self, cmd):
        if self.active is None:
            if cmd["type"] == "drive":
                self._begin_drive(cmd["distance_m"], cmd["speed_mps"])
            else:
                self._begin_turn(cmd["angle_rad"], cmd["rate_rps"])
            return

        if self.active["type"] == cmd["type"]:
            # Replace ongoing command of same type
            if cmd["type"] == "drive":
                self._begin_drive(cmd["distance_m"], cmd["speed_mps"])
            else:
                self._begin_turn(cmd["angle_rad"], cmd["rate_rps"])
        else:
            # Different type
            self.queue = [cmd]

    def _begin_drive(self, distance_m, speed_mps):
        self.active = {
            "type": "drive",
            "target_m": abs(distance_m),
            "sign": -1.0 if distance_m < 0 else 1.0,
            "speed_mps": abs(speed_mps),
            "baseline_total": self._dist_total,
            "started_at": time.monotonic(),
        }
        self.get_logger().info(f"DriveDistance start: {distance_m:.3f} m @ {speed_mps:.2f} m/s")

    def _begin_turn(self, angle_rad, rate_rps):
        self.active = {
            "type": "turn",
            "target_yaw": self._last_yaw + angle_rad,
            "rate_rps": abs(rate_rps),
            "dir": -1.0 if angle_rad < 0 else 1.0,
            "started_at": time.monotonic(),
        }
        self.get_logger().info(f"TurnAngle start: {math.degrees(angle_rad):.1f} deg @ {math.degrees(rate_rps):.1f} deg/s")

    def _stop_active(self, clear_queue=False):
        # Publish zero once, clear state
        stop = Twist()
        self.cmd_pub.publish(stop)
        self.active = None
        if clear_queue:
            self.queue = []
        self.get_logger().info("STOP: cleared active command" + (" and queue" if clear_queue else ""))

    def _angle_wrap(self, a):
        return (a + math.pi) % (2 * math.pi) - math.pi # Wrap to [-pi, pi]

    def _control_step(self):
        
        if self.active is None:
            return

        twist = Twist()

        if self.active["type"] == "drive":
            baseline = self.active["baseline_total"]
            traveled = max(0.0, self._dist_total - baseline)
            remaining = self.active["target_m"] - traveled

            if remaining <= 0.02:  # goal tolerance (2 cm)
                self._stop_active(clear_queue=False)
                if self.queue:
                    nxt = self.queue.pop(0)
                    self._enqueue_or_start(nxt)  
                return

            # constant forward/backward speed
            v = max(0.0, min(self._max_lin, float(self.active["speed_mps"])))
            twist.linear.x = self.active["sign"] * v
            self.get_logger().info(f"twist.linear.x: {twist.linear.x}")


        elif self.active["type"] == "turn":
            err = self._angle_wrap(self.active["target_yaw"] - self._last_yaw)
            if abs(err) <= math.radians(0.5):  # 0.5° tolerance
                self._stop_active(clear_queue=False)
                if self.queue:
                    nxt = self.queue.pop(0)
                    self._enqueue_or_start(nxt)
                return

            # constant yaw rate
            w = max(0.0, min(self._max_ang, float(self.active["rate_rps"])))
            twist.angular.z = self.active["dir"] * w
            self.get_logger().info(f"twist.linear.z: {twist.linear.z}")

        # Publish one step of motion
        self.cmd_pub.publish(twist)

    

    def poll_tc(self):
        try:
            while True:
                pkt, _ = self.tc_sock.recvfrom(1024)
                if not pkt:
                    break

                cmd_id = pkt[0]

                if cmd_id == 1:  # DriveDistance: [ID][distance_m][speed_mps]
                    if len(pkt) < 1 + 8:
                        self.get_logger().warn("DriveDistance packet too short")
                        continue
                    distance_m, speed_mps = struct.unpack('>ff', pkt[1:9])
                    speed_mps = abs(speed_mps) if speed_mps != 0.0 else 10.0 # 0.3
                    speed_mps = max(0.05, min(self._max_lin, speed_mps))
                    self._enqueue_or_start({
                        "type": "drive",
                        "distance_m": float(distance_m),
                        "speed_mps":  float(speed_mps),
                    })

                elif cmd_id == 2:  # TurnAngle: [ID][angle_deg][rate_degps]
                    if len(pkt) < 1 + 8:
                        self.get_logger().warn("TurnAngle packet too short")
                        continue
                    angle_deg, rate_degps = struct.unpack('>ff', pkt[1:9])
                    angle_rad = math.radians(float(angle_deg)) # Convert UI degrees to radians
                    rate_rps  = math.radians(float(rate_degps if rate_degps != 0.0 else 30.0))
                    rate_rps  = max(0.05, min(self._max_ang, abs(rate_rps)))
                    self._enqueue_or_start({
                        "type": "turn",
                        "angle_rad": float(angle_rad),
                        "rate_rps":  float(rate_rps),
                    })

                elif cmd_id == 0:  # Stop: [ID]
                    self._stop_active(clear_queue=True)

                else:
                    self.get_logger().warn(f"Unknown TC id {cmd_id:02X}, len={len(pkt)}; ignoring")
        except BlockingIOError:
            pass

        self._control_step()



def main():
    rclpy.init()
    node = YamcsRosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
