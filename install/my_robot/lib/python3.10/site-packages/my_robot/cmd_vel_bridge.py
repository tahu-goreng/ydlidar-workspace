#!/usr/bin/env python3
"""
====================================================
CMD_VEL_BRIDGE (SAFE FILTER)
Reactive Obstacle Avoidance (LiDAR-based)
====================================================
Input  : /cmd_vel
Output : /cmd_vel_safe

- Pass-through cmd_vel when path is clear
- Slow down and optionally steer when obstacle ahead
- Stop + rotate when too close

IMPORTANT FIX:
- LaserScan subscription uses BEST_EFFORT QoS to match LiDAR drivers
====================================================
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # ==============================
        # Parameters
        # ==============================
        self.declare_parameter('min_distance_stop', 0.3)   # meter
        self.declare_parameter('slowdown_start', 1.20)      # meter
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('input_cmd', '/cmd_vel')
        self.declare_parameter('output_cmd', '/cmd_vel_safe')

        # Turn rates when avoiding
        self.declare_parameter('turn_rate_stop', 0.80)      # rad/s
        self.declare_parameter('turn_rate_slow', 0.60)      # rad/s

        # Sector definitions (deg) in robot frame:
        # 0 deg = front, +deg = left, -deg = right
        self.declare_parameter('front_sector', [-15.0, 15.0])
        self.declare_parameter('left_sector',  [30.0, 90.0])
        self.declare_parameter('right_sector', [-90.0, -30.0])

        self.min_stop       = float(self.get_parameter('min_distance_stop').value)
        self.slowdown_start = float(self.get_parameter('slowdown_start').value)

        # Safety clamp: slowdown_start must be > min_stop
        if self.slowdown_start <= self.min_stop + 1e-6:
            self.get_logger().warn("slowdown_start <= min_stop. Adjusting slowdown_start = min_stop + 0.2")
            self.slowdown_start = self.min_stop + 0.2

        self.turn_rate_stop = float(self.get_parameter('turn_rate_stop').value)
        self.turn_rate_slow = float(self.get_parameter('turn_rate_slow').value)

        self.scan_topic  = self.get_parameter('scan_topic').value
        self.input_cmd   = self.get_parameter('input_cmd').value
        self.output_cmd  = self.get_parameter('output_cmd').value

        self.front_sector = self.get_parameter('front_sector').value
        self.left_sector  = self.get_parameter('left_sector').value
        self.right_sector = self.get_parameter('right_sector').value

        # ==============================
        # QoS (CRITICAL FIX)
        # ==============================
        qos_laser = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ==============================
        # ROS Interfaces
        # ==============================
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self.cb_scan, qos_laser
        )

        self.sub_cmd = self.create_subscription(
            Twist, self.input_cmd, self.cb_cmd, qos_cmd
        )

        self.pub_cmd = self.create_publisher(
            Twist, self.output_cmd, qos_cmd
        )

        self.latest_scan: LaserScan | None = None
        self.latest_cmd = Twist()

        self.timer = self.create_timer(0.05, self.update)  # 20 Hz
        self.last_log = None

        self.get_logger().info("CMD_VEL_BRIDGE: started (QoS fixed: /scan BEST_EFFORT).")

    # =================================================
    # Callbacks
    # =================================================
    def cb_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def cb_cmd(self, msg: Twist):
        self.latest_cmd = msg

    # =================================================
    # Helper: sanitize range value
    # =================================================
    @staticmethod
    def _valid_range(r: float, rmin: float, rmax: float) -> bool:
        # Some drivers can output 0.0 or out-of-range. Treat as invalid.
        if math.isnan(r) or math.isinf(r):
            return False
        if r <= 0.0:
            return False
        if r < rmin or r > rmax:
            return False
        return True

    # =================================================
    # Helper: minimum distance in sector (deg)
    # Works for typical 360° LaserScan. Uses angle_min/increment.
    # =================================================
    def sector_min(self, angle_start_deg: float, angle_end_deg: float) -> float:
        scan = self.latest_scan
        if scan is None:
            return float('inf')

        n = len(scan.ranges)
        if n == 0:
            return float('inf')

        # Normalize so start <= end
        a0 = float(angle_start_deg)
        a1 = float(angle_end_deg)
        if a0 > a1:
            a0, a1 = a1, a0

        # Convert degrees to radians
        a0r = math.radians(a0)
        a1r = math.radians(a1)

        # LaserScan angles
        amin = float(scan.angle_min)
        inc  = float(scan.angle_increment)

        if inc <= 0.0:
            return float('inf')

        # Compute indices; clamp
        i0 = int((a0r - amin) / inc)
        i1 = int((a1r - amin) / inc)

        # Clamp to valid range
        i0 = max(0, min(n - 1, i0))
        i1 = max(0, min(n - 1, i1))

        if i0 > i1:
            i0, i1 = i1, i0

        rmin = float(scan.range_min)
        rmax = float(scan.range_max)

        vals = []
        for r in scan.ranges[i0:i1 + 1]:
            if self._valid_range(r, rmin, rmax):
                vals.append(r)

        return min(vals) if vals else float('inf')

    # =================================================
    # Main Logic
    # =================================================
    def update(self):
        # If scan not ready, publish STOP for safety
        if self.latest_scan is None:
            msg = Twist()
            self.pub_cmd.publish(msg)
            return

        # --- Read sectors ---
        front = self.sector_min(self.front_sector[0], self.front_sector[1])
        left  = self.sector_min(self.left_sector[0],  self.left_sector[1])
        right = self.sector_min(self.right_sector[0], self.right_sector[1])

        # Input command (from teleop or other node)
        v_in = float(self.latest_cmd.linear.x)
        w_in = float(self.latest_cmd.angular.z)

        v_out = v_in
        w_out = w_in  # keep user steering by default

        # =============================
        # Decision Logic (override only when obstacle)
        # =============================
        if front < self.min_stop:
            # TOO CLOSE → STOP + ROTATE
            v_out = 0.0
            w_out = self.turn_rate_stop if left > right else -self.turn_rate_stop

        elif front < self.slowdown_start:
            # SLOW DOWN + STEER AWAY
            scale = (front - self.min_stop) / (self.slowdown_start - self.min_stop)
            scale = max(min(scale, 1.0), 0.1)  # keep minimal motion if user wants forward

            v_out = v_in * scale
            # if user already turning, keep that direction; else choose freer side
            if abs(w_in) < 0.05:
                w_out = self.turn_rate_slow if left > right else -self.turn_rate_slow

        # else: CLEAR PATH → pass-through cmd_vel (v_out=v_in, w_out=w_in)

        # =============================
        # Publish Safe Command
        # =============================
        msg = Twist()
        msg.linear.x = float(v_out)
        msg.angular.z = float(w_out)
        self.pub_cmd.publish(msg)

        # =============================
        # Debug (2 Hz)
        # =============================
        now = self.get_clock().now()
        if self.last_log is None or (now - self.last_log).nanoseconds > 5e8:
            self.get_logger().info(
                f"Front={front:.2f}  Left={left:.2f}  Right={right:.2f} | "
                f"vin={v_in:.2f} win={w_in:.2f} -> v={v_out:.2f} w={w_out:.2f}"
            )
            self.last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

