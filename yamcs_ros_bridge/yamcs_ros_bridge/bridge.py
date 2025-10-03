#!/usr/bin/env python3
import json, math, socket, struct, time
import os, uuid
from pathlib import Path
from datetime import datetime, timezone
from zoneinfo import ZoneInfo
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
from sensor_msgs.msg import Image  
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import requests 


# YAMCS_PARAM_HOST = '127.0.0.1'
YAMCS_PARAM_HOST = '10.0.0.186'
YAMCS_PARAM_PORT = 10050      # udp-params-in - telemetry
YAMCS_TC_LISTEN_PORT = 10051  # udp-tc-out - commands

# Photo upload constants
PHOTO_ROOT = Path("~/ros_ws/leorover_photos").expanduser()
PHOTO_ROOT.mkdir(parents=True, exist_ok=True)  
# CAM_DEVICE_INDEX = 0
BUCKET_NAME = "leocam" #"leorover_capture"
YAMCS_HTTP = "http://10.0.0.186:8090"
HTTP_USER = "admin"
HTTP_PASSWORD = "admin"

# CFDP constants - not using it right now
CFDP_REMOTE_ROVER = 5                             
CFDP_LOCAL_GROUND = 11                              
CFDP_UDP_OUT_HOST = "10.0.0.186"                   
CFDP_UDP_IN_PORT  = 10060 # Telemetry                         
CFDP_UDP_OUT_PORT = 10061 # Commands                      
USE_CFDP = False  


class YamcsRosBridge(Node):
    def __init__(self):
        super().__init__('yamcs_ros_bridge')

        # Publishers
        self.sub = self.create_subscription(Odometry, '/wheel_odom', self._odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Battery
        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_sensor.history = HistoryPolicy.KEEP_LAST
        self.batt_sub = self.create_subscription(Float32, '/firmware/battery_averaged', self._batt_cb, qos_sensor)

        # Photo 
        self.bridge = CvBridge()                                 
        self._last_frame = None                                  
        self.img_sub = self.create_subscription(Image, '/camera/image_color', self._img_cb, 10)

        # UDP sockets
        self.param_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tc_sock.bind(('0.0.0.0', YAMCS_TC_LISTEN_PORT))
        self.tc_sock.setblocking(False)

        # Telemetry 
        self._prev_xy = None
        self._last_yaw = 0.0
        self._lin = 0.0
        self._ang = 0.0
        self._dist_total = 0.0
        self._battery = 0.0

        # Automatic capture states
        self._timed_active = False
        self._timed_session_id = None
        self._timed_seq = 0
        self._timed_interval = None
        self._timed_command_time_utc = None
        self._timed_next_due_mono = None
        self._timed_end_mono = None
        self.create_timer(0.02, self._timed_step) # timer

        # Timer poll for commands 
        self.create_timer(0.02, self.poll_tc) 

        # Commands control states
        self.active = None     
        self.queue  = []       
        self._max_lin = 10.0 #    
        self._max_ang = 10.0  # 1.0

        self.get_logger().info("Bridge Established")


    def _io_worker(self):
        while True:
            task = self._io_q.get()
            if task is None:
                break
            img_path, meta_path, meta = task
            try:
                self._upload_via_http(img_path, meta_path, meta)
            except Exception as e:
                self.get_logger().error(f"Uploader error: {e}")
            finally:
                self._io_q.task_done()

    ######## Telemetry #################################################

    def _batt_cb(self, msg: Float32):
        self._battery = float(msg.data) if not math.isnan(msg.data) else 0.0
        # self.get_logger().info(f"battery msg: {msg}")

    def _odom_cb(self, msg: Odometry):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        # Yaw from quaternion 
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self._last_yaw = float(math.atan2(siny_cosp, cosy_cosp))
        # self._last_yaw = float(yaw)

        self._lin = msg.twist.twist.linear.x
        self._ang = msg.twist.twist.angular.z

        # integrate distance in 2D
        if self._prev_xy is not None:
            dx = x - self._prev_xy[0]
            dy = y - self._prev_xy[1]
            self._dist_total += math.hypot(dx, dy)
        self._prev_xy = (x, y)

        # self.get_logger().info(f"X: {x}")

        payload = {
            "parameter": [
                {"id": {"name": "/leorover/ODOM/x"}, "engValue": {"type":"DOUBLE","doubleValue": float(x)}},
                {"id": {"name": "/leorover/ODOM/y"}, "engValue": {"type":"DOUBLE","doubleValue": float(y)}},
                {"id": {"name": "/leorover/ODOM/yaw"}, "engValue": {"type":"DOUBLE","doubleValue": float(self._last_yaw)}},
                {"id": {"name": "/leorover/ODOM/linear_speed"}, "engValue": {"type":"DOUBLE","doubleValue": float(self._lin)}},
                {"id": {"name": "/leorover/ODOM/angular_speed"}, "engValue": {"type":"DOUBLE","doubleValue": float(self._ang)}},
                {"id": {"name": "/leorover/ODOM/distance_total"}, "engValue": {"type":"DOUBLE","doubleValue": float(self._dist_total)}},
                {"id": {"name": "/leorover/ODOM/battery"}, "engValue": {"type":"DOUBLE","doubleValue": float(self._battery)}},

                # {"id":{"name":"/leorover/ODOM/last_photo_bucket"}, "engValue":{"type":"STRING","stringValue": ""}},
                # {"id":{"name":"/leorover/ODOM/last_photo_object"}, "engValue":{"type":"STRING","stringValue": ""}},
                # {"id":{"name":"/leorover/ODOM/last_photo_time_est"}, "engValue":{"type":"STRING","stringValue": ""}},
                # {"id":{"name":"/leorover/ODOM/last_photo_time_utc"}, "engValue":{"type":"STRING","stringValue": ""}},
            ]
        }
        data = json.dumps(payload).encode('utf-8')
        self.param_sock.sendto(data, (YAMCS_PARAM_HOST, YAMCS_PARAM_PORT))
        self.get_logger().info(f"Data: {data}")


    ########## Drive Commands functions #############################################

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

    def angle_wrap(self, a):
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
            err = self.angle_wrap(self.active["target_yaw"] - self._last_yaw)
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

    
    ########## Take Photo Commands functions #########################################################

    def _img_cb(self, msg: Image):       

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._last_frame = frame
        except Exception as e:
            self.get_logger().warn(f"CV Bridge error: {e}")

    def _make_session_id(self, single: bool) -> tuple[str, str]:
        """
          timed:  YYYYMMDDThhmmssZ-xxxxxx
          single: single-YYYYMMDDThhmmssZ-xxxxxx
        """
        t = datetime.now(timezone.utc)
        ts = t.strftime("%Y%m%dT%H%M%SZ")  # seconds + 'Z'
        suffix = uuid.uuid4().hex[:6]
        sid = f"{ts}-{suffix}"
        if single:
            sid = f"single-{sid}"
        return sid, t.isoformat()

    def _take_photo_and_write_files(self, session_id: str, seq: int|None,interval_sec: float|None, command_time_utc: str):
        
        # Time
        now_utc = datetime.now(timezone.utc)
        est = ZoneInfo("America/Toronto")
        now_est = now_utc.astimezone(est)                            
        ts_est = now_est.strftime("%Y-%m-%d %H:%M:%S.%f %Z")        
        ts_utc = now_utc.isoformat()  

        # File name 
        base_id = session_id[len("single-"):] if session_id.startswith("single-") else session_id
        if seq is None:
            stem = base_id
        else:
            stem = f"{base_id}__{seq:06d}"
        img_path = PHOTO_ROOT / f"{stem}.jpg"
        meta_path = PHOTO_ROOT / f"{stem}.json"
        # base = now_utc.strftime("%Y%m%dT%H%M%S_%f")[:-3]
        # img_path = PHOTO_ROOT / f"{base}.jpg"
        # meta_path = PHOTO_ROOT / f"{base}.json" 

        # Check Frame and write to path
        if self._last_frame is None:
            raise RuntimeError("Image read failed")

        if not cv2.imwrite(str(img_path), self._last_frame):                    
            raise RuntimeError("Image write failed")

        # Telemetry and write in json
        meta = {                                                     
            "time_est": ts_est,
            "time_utc": ts_utc,
            "x": float(self._prev_xy[0] if self._prev_xy else 0.0),
            "y": float(self._prev_xy[1] if self._prev_xy else 0.0),
            "yaw": float(self._last_yaw),
            "linear_speed": float(self._lin),    
            "angular_speed": float(self._ang),   
            "distance_total": float(self._dist_total),
            "battery": float(self._battery if self._battery else 0.0),
            "bucket": BUCKET_NAME,         
            "object_image": img_path.name, 
            "object_json":  meta_path.name,

            "session_id": session_id,
            "seq": 0 if seq is None else int(seq),
            "interval_sec": None if interval_sec is None else float(interval_sec),
            "command_time_utc": command_time_utc,
        }

        with open(meta_path, "w") as f:
            json.dump(meta, f, indent=2)

        self.get_logger().info(f"Saved {img_path.name} & {meta_path.name}")  

        return img_path, meta_path, meta 


    def _publish_last_photo_params(self, meta: dict):
        # Telemetry parameters 
        params = [                                                   
            {"id":{"name":"/leorover/ODOM/last_photo_bucket"}, "engValue":{"type":"STRING","stringValue": meta["bucket"]}},
            {"id":{"name":"/leorover/ODOM/last_photo_object"}, "engValue":{"type":"STRING","stringValue": meta["object_image"]}},
            {"id":{"name":"/leorover/ODOM/last_photo_time_est"}, "engValue":{"type":"STRING","stringValue": meta["time_est"]}},
            {"id":{"name":"/leorover/ODOM/last_photo_time_utc"}, "engValue":{"type":"STRING","stringValue": meta["time_utc"]}},
        ]

        payload = {"parameter": params}
        self.param_sock.sendto(json.dumps(payload).encode("utf-8"), (YAMCS_PARAM_HOST, YAMCS_PARAM_PORT))  

    def _upload_via_http(self, img_path: Path, meta_path: Path, meta: dict):

        # Post Image
        with open(img_path, "rb") as f:
            r = requests.post(
                f"{YAMCS_HTTP}/api/storage/buckets/{BUCKET_NAME}/objects/{img_path.name}",
                data=f,
                headers={"Content-Type": "image/jpeg"},  
                auth=(HTTP_USER, HTTP_PASSWORD),
                timeout=15,
            )
        r.raise_for_status()

        # Post JSON
        with open(meta_path, "rb") as f:
            r = requests.post(
                f"{YAMCS_HTTP}/api/storage/buckets/{BUCKET_NAME}/objects/{meta_path.name}",
                data=f,
                headers={"Content-Type": "application/json"},
                auth=(HTTP_USER, HTTP_PASSWORD),
                timeout=15,
            )
        r.raise_for_status()

        # URL for widget
        url = f"{YAMCS_HTTP}/api/storage/buckets/{BUCKET_NAME}/objects/{img_path.name}"
        meta["url"] = url

        # Parameter for last_photo_url
        payload = {"parameter":[
            {"id":{"name":"/leorover/ODOM/last_photo_url"},
             "engValue":{"type":"STRING","stringValue": url}}
        ]}
        self.param_sock.sendto(json.dumps(payload).encode("utf-8"),
                               (YAMCS_PARAM_HOST, YAMCS_PARAM_PORT))

        # Post Yamcs Event 
        evt = {
            "type": "INFO",
            "source": "leorover.camera",
            "message": f"Photo saved: {img_path.name}",
            "time": meta["time_utc"],
            "extra": {"bucket": BUCKET_NAME, "image": img_path.name, "json": meta_path.name}
        }
        r = requests.post(f"{YAMCS_HTTP}/api/archive/{'leorover'}/events",
                          json=evt, auth=(HTTP_USER, HTTP_PASSWORD))
        ########################## replace 'leorover' with your actual instance name in the URL    ################## IMPORTANT
        r.raise_for_status()

        self.get_logger().info(f"Uploaded to bucket {BUCKET_NAME} and posted event")



    def _send_via_cfdp(self, img_path: Path, meta_path: Path):
        from cfdppy.handler.source import SourceHandler
        from cfdppy.request import PutRequest
        import socket as _s

        # send one PDU datagram to Yamcs udp-cfdp-in 
        def send_pdu(raw: bytes):
            s = _s.socket(_s.AF_INET, _s.SOCK_DGRAM)
            s.sendto(raw, (CFDP_UDP_OUT_HOST, CFDP_UDP_IN_PORT))
            s.close()

        # Send image
        put_img = PutRequest(
            source_entity_id=CFDP_REMOTE_ROVER,      
            dest_entity_id=CFDP_LOCAL_GROUND,        
            source_file=str(img_path),              
            dest_path=f"{img_path.name}",           
            acknowledged=False,                      
            closure_requested=True                  
        )
        sh = SourceHandler()                        
        sh.put_request(put_img)                    
        while True:
            pdu = sh.get_next_packet()              
            if pdu is None:
                break
            send_pdu(pdu.to_bytes())               

        # Send JSON sidecar
        put_json = PutRequest(
            source_entity_id=CFDP_REMOTE_ROVER,
            dest_entity_id=CFDP_LOCAL_GROUND,
            source_file=str(meta_path),
            dest_path=f"{meta_path.name}",
            acknowledged=False,
            closure_requested=True
        )
        sh = SourceHandler()
        sh.put_request(put_json)
        while True:
            pdu = sh.get_next_packet()
            if pdu is None:
                break
            send_pdu(pdu.to_bytes())

        self.get_logger().info("CFDP PDUs sent for image+json")



    ############ Automatic Capture Image Functions ################

    def _start_timed_capture(self, interval_sec: float, duration_sec: float):
        interval = max(0.1, float(interval_sec))
        mono_now = time.monotonic()

        self._timed_active = True
        self._timed_session_id, self._timed_command_time_utc = self._make_session_id(single=False)
        self._timed_seq = 0
        self._timed_interval = interval
        self._timed_next_due_mono = mono_now  
        self._timed_end_mono = (mono_now + float(duration_sec)) if duration_sec > 0 else None

        self.get_logger().info(
            f"TimedCapture START: session={self._timed_session_id}, interval={interval:.3f}s, "
            f"duration={'∞' if self._timed_end_mono is None else f'{duration_sec:.1f}s'}"
        )

    def _stop_timed_capture(self, reason: str = "manual"):
        if not self._timed_active:
            return
        self.get_logger().info(f"TimedCapture STOP ({reason}): session={self._timed_session_id}, last_seq={self._timed_seq-1}")
        self._timed_active = False
        self._timed_session_id = None
        self._timed_seq = 0
        self._timed_interval = None
        self._timed_command_time_utc = None
        self._timed_next_due_mono = None
        self._timed_end_mono = None

    def _timed_step(self):
        if not self._timed_active:
            return

        mono_now = time.monotonic()

        if self._timed_end_mono is not None and mono_now >= self._timed_end_mono:
            self._stop_timed_capture(reason="duration")
            return

        if mono_now >= self._timed_next_due_mono:
            try:
                img_path, meta_path, meta = self._take_photo_and_write_files(
                    session_id=self._timed_session_id,
                    seq=self._timed_seq,
                    interval_sec=self._timed_interval,
                    command_time_utc=self._timed_command_time_utc,
                )
                self._publish_last_photo_params(meta)
                self._upload_via_http(img_path, meta_path, meta)
            except Exception as e:
                self.get_logger().error(f"TimedCapture shot failed: {e}")

            # schedule next shot
            self._timed_seq += 1
            self._timed_next_due_mono = mono_now + self._timed_interval


    ########### Function for Commands ###################################

    def poll_tc(self):
        try:
            while True:
                pkt, _ = self.tc_sock.recvfrom(4096) # 1024
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

                elif cmd_id == 3:
                    try:
                        # img_path, meta_path, meta = self._take_photo_and_write_files() # Old
                        session_id, cmd_time = self._make_session_id(single=True)
                        img_path, meta_path, meta = self._take_photo_and_write_files(session_id=session_id, seq=None, interval_sec=None, command_time_utc=cmd_time)

                        self._publish_last_photo_params(meta)
                        # if USE_CFDP:
                        #     self._send_via_cfdp(img_path, meta_path)
                        # else:
                        self._upload_via_http(img_path, meta_path, meta)
                    except Exception as e:
                        self.get_logger().error(f"TakePhoto failed: {e}")

                elif cmd_id == 4:  # [4][interval_sec][duration_sec]
                    interval_sec = None
                    duration_sec = 0.0
                    payload_len = len(pkt) - 1  # subtract 1 for the cmd_id byte

                    if payload_len >= 8:
                        interval_sec, duration_sec = struct.unpack('>ff', pkt[1:9])
                    elif payload_len >= 4:
                        (interval_sec,) = struct.unpack('>f', pkt[1:5])

                    if interval_sec is None:
                        self.get_logger().warn("StartTimedCapture missing interval_sec")
                        continue

                    try:
                        self._start_timed_capture(interval_sec=float(interval_sec), duration_sec=float(duration_sec))
                    except Exception as e:
                        self.get_logger().error(f"StartTimedCapture failed: {e}")

                elif cmd_id == 5: # To stop automatic photo cmd
                    self._stop_timed_capture(reason="manual")

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
