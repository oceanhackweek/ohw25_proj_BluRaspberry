#!/usr/bin/env python3
import json
import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time as RosTime

from std_msgs.msg import String, Float64, UInt32
from sensor_msgs.msg import NavSatFix, NavSatStatus, BatteryState

DEG_LAT_M = 111_111.0  # meters per degree latitude (approx)
def meters_to_deg_lat(m): return m / DEG_LAT_M
def meters_to_deg_lon(m, lat_deg): return m / (DEG_LAT_M * max(0.1, math.cos(math.radians(lat_deg))))

class MockUUVNode(Node):
    """
    Publishes the exact topics your DroneBridge subscribes to,
    and listens to /frontend/* topics from your server.
    """

    def __init__(self):
        super().__init__('mock_uuv')

        # ---------------- Params (override in launch/CLI) ----------------
        self.declare_parameter('start_lat', 33.810313)
        self.declare_parameter('start_lon', -118.393867)
        self.declare_parameter('start_heading_deg', 90.0)     # east
        self.declare_parameter('speed_mps', 0.8)               # cruise speed
        self.declare_parameter('rel_alt_m', -2.0)              # UUV depth = negative altitude
        self.declare_parameter('battery_start_v', 15.8)        # 4S full-ish
        self.declare_parameter('battery_min_v', 13.2)
        self.declare_parameter('battery_hours', 2.5)           # linear drain over N hours while moving
        self.declare_parameter('num_cameras', 1)
        self.declare_parameter('gps_sat_min', 8)
        self.declare_parameter('gps_sat_max', 14)
        self.declare_parameter(
            'capabilities_json',
            json.dumps({
                "hardware": {
                    "camera": False,
                    "sonar": True
                },
                "missions": [
                    {"available": True,  "geometry_type": "polyline", "name": "DEPTHMOWER"}
                ],
                "perception_modules": {
                    "obstacle_avoidance": {"active": True,  "togglable": False},
                    "oxygen_mapper":   {"active": False, "togglable": True},
                    "bathymetry_mapper":     {"active": False, "togglable": True}
                }
            })
        )

        # resolved params
        self.lat = float(self.get_parameter('start_lat').value)
        self.lon = float(self.get_parameter('start_lon').value)
        self.heading_deg = float(self.get_parameter('start_heading_deg').value)
        self.speed_mps = float(self.get_parameter('speed_mps').value)
        self.rel_alt_m = float(self.get_parameter('rel_alt_m').value)
        self.batt_v = float(self.get_parameter('battery_start_v').value)
        self.batt_min_v = float(self.get_parameter('battery_min_v').value)
        self.batt_hours = float(self.get_parameter('battery_hours').value)
        self.num_cameras = int(self.get_parameter('num_cameras').value)
        self.gps_sat_min = int(self.get_parameter('gps_sat_min').value)
        self.gps_sat_max = int(self.get_parameter('gps_sat_max').value)
        self.capabilities_json = str(self.get_parameter('capabilities_json').value)

        # sim state
        self.distance_m = 0.0
        self.status = "READY"
        self.last_tick = time.time()
        self.log_level = "info"
        self.mission_active = False
        self.estopped = False
        self.home = (self.lat, self.lon)
        self.last_log_ms = 0
        self.speed_filter = self.speed_mps  # used for reporting

        # ---------------- Publishers ----------------
        self.pub_capabilities = self.create_publisher(String, '/task_manager/capabilities', 10)
        self.pub_status       = self.create_publisher(String, '/task_manager/rest_status', 10)
        self.pub_log          = self.create_publisher(String, '/task_manager/rest_log', 10)

        self.pub_heading      = self.create_publisher(Float64,  '/mavros/global_position/compass_hdg', qos_profile_sensor_data)
        self.pub_navsat       = self.create_publisher(NavSatFix, '/mavros/global_position/global',      qos_profile_sensor_data)
        self.pub_battery      = self.create_publisher(BatteryState, '/mavros/battery', qos_profile_sensor_data)
        self.pub_sats         = self.create_publisher(UInt32,   '/mavros/global_position/raw/satellites', qos_profile_sensor_data)
        self.pub_rel_alt      = self.create_publisher(Float64,  '/mavros/global_position/rel_alt', qos_profile_sensor_data)

        # ---------------- Subscriptions ----------------
        self.sub_run_mission  = self.create_subscription(String, '/frontend/run_mission', self.on_run_mission, 10)
        self.sub_toggle_mod   = self.create_subscription(String, '/frontend/toggle_module', self.on_toggle_module, 10)
        self.sub_emergency    = self.create_subscription(String, '/frontend/emergency', self.on_emergency, 10)
        self.sub_remote_id    = self.create_subscription(String, '/frontend/remote_id', self.on_remote_id, 10)

        # ---------------- Timers ----------------
        # High-rate vehicle sensors
        self.create_timer(0.10, self.tick_10hz)   # 10 Hz
        # Status + log cadence
        self.create_timer(1.0, self.tick_1hz)     # 1 Hz
        # Capabilities heartbeat
        self.create_timer(10.0, self.publish_capabilities)  # 0.1 Hz

        # Publish initial capabilities immediately
        self.publish_capabilities()
        self.get_logger().info("MockUUVNode up — publishing MAVROS/task_manager topics and listening on /frontend/*")

    # ----------------- Helpers -----------------
    def now_stamp(self) -> RosTime:
        return self.get_clock().now().to_msg()

    def publish_capabilities(self):
        self.pub_capabilities.publish(String(data=self.capabilities_json))

    def publish_status(self):
        status = {
            "status": self.status,
            "num_cameras": self.num_cameras,
            "distance": round(self.distance_m, 2),
            "speed": round(self.speed_filter, 2),
        }
        self.pub_status.publish(String(data=json.dumps(status)))

    def publish_log(self, message: str, level: str = "info"):
        self.last_log_ms = int(time.time() * 1000)
        payload = {"message": message, "level": level}
        self.pub_log.publish(String(data=json.dumps(payload)))

    def publish_nav(self):
        # Heading
        self.pub_heading.publish(Float64(data=float(self.heading_deg)))

        # GPS fix
        fix = NavSatFix()
        fix.header.stamp = self.now_stamp()
        fix.header.frame_id = "base_link"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = float(self.lat)
        fix.longitude = float(self.lon)
        fix.altitude = float(self.rel_alt_m)  # underwater depth as negative altitude
        # simple covariance/unknown
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub_navsat.publish(fix)

        # Relative altitude (depth)
        self.pub_rel_alt.publish(Float64(data=float(self.rel_alt_m)))

        # Satellites
        sats = random.randint(self.gps_sat_min, self.gps_sat_max)
        self.pub_sats.publish(UInt32(data=sats))

    def publish_battery(self, dt):
        # Linear drain when moving
        batt_span = max(0.1, self.batt_v - self.batt_min_v)
        drain_rate_v_per_sec = batt_span / (self.batt_hours * 3600.0)
        moving = (self.speed_mps > 0.05) and not self.estopped
        if moving:
            self.batt_v = max(self.batt_min_v, self.batt_v - drain_rate_v_per_sec * dt)

        msg = BatteryState()
        msg.header.stamp = self.now_stamp()
        msg.voltage = float(self.batt_v)
        msg.current = float(-2.0 if moving else -0.3)  # rough guess
        msg.percentage = max(0.0, min(1.0, (self.batt_v - self.batt_min_v) / (self.get_parameter('battery_start_v').value - self.batt_min_v)))
        self.pub_battery.publish(msg)

    def sim_motion(self, dt):
        # no motion if estopped or paused
        if self.estopped or self.status in ("paused", "rtl_landed"):
            self.speed_filter = max(0.0, 0.9 * self.speed_filter)
            return

        # mild random walk if idle; steady if mission_active
        speed_cmd = self.speed_mps if self.mission_active else 0.2
        # tiny heading wander when idle
        if not self.mission_active:
            self.heading_deg += random.uniform(-2.0, 2.0) * dt

        # clamp heading
        while self.heading_deg < 0: self.heading_deg += 360.0
        while self.heading_deg >= 360.0: self.heading_deg -= 360.0

        # integrate
        vx = speed_cmd * math.cos(math.radians(self.heading_deg))
        vy = speed_cmd * math.sin(math.radians(self.heading_deg))
        dlon = meters_to_deg_lon(vx * dt, self.lat)
        dlat = meters_to_deg_lat(vy * dt)

        self.lon += dlon
        self.lat += dlat
        self.distance_m += max(0.0, speed_cmd * dt)

        # report speed with a little smoothing
        self.speed_filter = 0.8 * self.speed_filter + 0.2 * speed_cmd

    # ----------------- Timers -----------------
    def tick_10hz(self):
        t = time.time()
        dt = min(0.2, max(0.0, t - self.last_tick))
        self.last_tick = t

        self.sim_motion(dt)
        self.publish_nav()
        self.publish_battery(dt)

    def tick_1hz(self):
        # keep status up to date
        if self.estopped:
            self.status = "FAILSAFE"
        elif self.mission_active:
            self.status = "MISSION"
        elif self.status not in ("PAUSE", "LAND"):
            self.status = "READY"

        self.publish_status()

        # periodic log heartbeat
        if random.random() < 0.25:
            self.publish_log(f"tick — lat={self.lat:.6f} lon={self.lon:.6f} spd={self.speed_filter:.2f}m/s", "info")

    # ----------------- Subscribers -----------------
    def on_run_mission(self, msg: String):
        # Expecting a JSON string, but tolerate raw text
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"mission": msg.data or "unknown"}

        self.mission_active = True
        self.estopped = False
        # Optionally accept a heading or path from payload
        hdg = payload.get("heading_deg")
        if isinstance(hdg, (int, float)):
            self.heading_deg = float(hdg)

        # Optional speed override
        spd = payload.get("speed_mps")
        if isinstance(spd, (int, float)) and spd >= 0.0:
            self.speed_mps = float(spd)

        self.publish_log(f"Mission start: {payload}", "info")
        self.get_logger().info(f"Received /frontend/run_mission: {payload}")

    def on_toggle_module(self, msg: String):
        # Flip capability enabled state by name
        try:
            caps = json.loads(self.capabilities_json)
        except Exception:
            caps = []
        target = msg.data.strip()
        changed = False
        for c in caps:
            if c.get("name") == target:
                c["enabled"] = not bool(c.get("enabled"))
                changed = True
                break
        if changed:
            self.capabilities_json = json.dumps(caps)
            self.publish_capabilities()
            self.publish_log(f"Toggled module '{target}'", "info")
        else:
            self.publish_log(f"Module '{target}' not found", "warn")
        self.get_logger().info(f"Received /frontend/toggle_module: {msg.data}")

    def on_emergency(self, msg: String):
        cmd = (msg.data or "").lower().strip()
        if cmd in ("estop", "e-stop", "kill"):
            self.estopped = True
            self.mission_active = False
            self.status = "e-stop"
            self.publish_log("E-STOP engaged", "error")
        elif cmd in ("pause", "hold"):
            self.mission_active = False
            self.status = "paused"
            self.publish_log("Paused", "warn")
        elif cmd in ("rtl", "return", "return_to_launch"):
            # simple RTL: turn toward home and keep moving until close
            self.mission_active = True
            self.estopped = False
            self.status = "rtl"
            self.publish_log("RTL initiated", "warn")
            self._point_toward_home()
        else:
            self.publish_log(f"Unknown emergency cmd '{cmd}'", "warn")
        self.get_logger().warn(f"Received /frontend/emergency: {cmd}")

    def _point_toward_home(self):
        # compute heading to home
        dlon_m = (self.home[1] - self.lon) * DEG_LAT_M * math.cos(math.radians(self.lat))
        dlat_m = (self.home[0] - self.lat) * DEG_LAT_M
        ang = math.degrees(math.atan2(dlat_m, dlon_m))
        # atan2(y, x) -> here y=dlat_m (north), x=dlon_m (east); heading east=0 -> convert
        hdg = (ang + 360.0) % 360.0
        self.heading_deg = hdg

        # if close, “land”
        dist = math.hypot(dlon_m, dlat_m)
        if dist < 5.0:
            self.mission_active = False
            self.status = "rtl_landed"
            self.publish_log("RTL complete", "info")

    def on_remote_id(self, msg: String):
        # Pass-through; just log reception
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"raw": msg.data}
        self.publish_log(f"Remote ID update: {payload}", "info")
        self.get_logger().info(f"Received /frontend/remote_id: {payload}")


def main():
    rclpy.init()
    node = MockUUVNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
