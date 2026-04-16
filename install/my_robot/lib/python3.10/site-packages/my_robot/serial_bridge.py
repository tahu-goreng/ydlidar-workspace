#!/usr/bin/env python3
"""
====================================================
ROS 2 Serial Bridge → ESP32-S3 (SAFE VERSION)
====================================================
ARM  : only when cmd_vel is received
DISARM : when cmd_vel timeout
NO AUTO MOVE AT STARTUP
====================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import threading
import time
import math


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # ==========================
        # Parameters
        # ==========================
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('cmd_timeout', 0.5)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baudrate').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        # ==========================
        # Serial
        # ==========================
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        self.get_logger().info(f"Serial opened {self.port}")

        # ❌ TIDAK ADA AUTO ARM
        self.armed = False
        self.last_cmd_time = time.time()

        # ==========================
        # ROS
        # ==========================
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_safe', self.cb_cmd, 10
        )

        self.odom_pub = self.create_publisher(
            Odometry, '/odom', 20
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # Safety timer
        self.create_timer(0.1, self._safety_check)

        # Serial reader thread
        self._stop = False
        self.reader_thread = threading.Thread(
            target=self._read_loop, daemon=True
        )
        self.reader_thread.start()

        self.get_logger().info("READY: waiting for cmd_vel (teleop)")

    # ==========================
    # Serial write
    # ==========================
    def _write(self, s: str):
        self.ser.write((s + "\n").encode())

    # ==========================
    # cmd_vel callback
    # ==========================
    def cb_cmd(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        self.last_cmd_time = time.time()

        # ARM only once
        if not self.armed:
            self._write("ARM")
            self.armed = True
            self.get_logger().info("ARMED")

        self._write(f"CMD {v:.3f} {w:.3f}")

    # ==========================
    # Safety timeout
    # ==========================
    def _safety_check(self):
        if self.armed and (time.time() - self.last_cmd_time > self.cmd_timeout):
            self._write("CMD 0 0")
            self._write("DISARM")
            self.armed = False
            self.get_logger().warn("DISARM (cmd_vel timeout)")

    # ==========================
    # Serial reader
    # ==========================
    def _read_loop(self):
        buf = ""
        while rclpy.ok() and not self._stop:
            data = self.ser.read(256).decode(errors='ignore')
            if not data:
                continue
            buf += data
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                self._handle_serial_line(line.strip())

    def _handle_serial_line(self, line: str):
        if not line.startswith("ODOM"):
            return

        parts = line.split()
        if len(parts) < 8:
            return

        try:
            x = float(parts[1])
            y = float(parts[2])
            th = float(parts[3])
            v = float(parts[4])
            w = float(parts[5])
        except ValueError:
            return

        now = self.get_clock().now().to_msg()

        # Odometry msg
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y

        qz = math.sin(th * 0.5)
        qw = math.cos(th * 0.5)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)

    # ==========================
    # Shutdown
    # ==========================
    def destroy_node(self):
        self._stop = True
        if self.armed:
            self._write("DISARM")
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

