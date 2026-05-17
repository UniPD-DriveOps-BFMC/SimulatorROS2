#!/usr/bin/env python3

import math
import random
from collections import deque

import json

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose


class SensorConverterNode(Node):
    """
    Publishes encoder and GPS topics that mirror the real BFMC car interface.

    Encoder (from /encoder_odom):
      /automobile/encoder/speed    Float32  [m/s]  instantaneous forward speed
      /automobile/encoder/distance Float32  [m]    cumulative odometer distance

    GPS (from /automobile/localisation_raw, Pose with ±15 cm noise applied by plugin):
      /automobile/localisation     String  {"x":float, "y":float, "z":float, "quality":int}
      — published with a simulated 1 s GPS receiver delay
    """

    GPS_DELAY_S = 1.0

    def __init__(self):
        super().__init__('sensor_converter')

        # --- Encoder ---
        self.sub_odom = self.create_subscription(
            Odometry, '/encoder_odom', self._odom_cb, 10)
        self.pub_speed = self.create_publisher(
            Float32, '/automobile/encoder/speed', 10)
        self.pub_distance = self.create_publisher(
            Float32, '/automobile/encoder/distance', 10)
        self._total_dist = 0.0
        self._last_pos = None

        # --- GPS ---
        self.sub_gps = self.create_subscription(
            Pose, '/automobile/localisation_raw', self._gps_cb, 10)
        self.pub_gps = self.create_publisher(
            String, '/automobile/localisation', 10)
        self._gps_buf: deque = deque()  # (wall_time_s, Pose)
        self.create_timer(0.05, self._flush_gps_buf)  # drain at 20 Hz

        # --- Traffic detection placeholder (topic advertised, no messages published) ---
        self.pub_traffic = self.create_publisher(
            String, '/traffic/detection', 10)

        self.get_logger().info('Sensor converter started.')
        self.get_logger().info(
            '  /encoder_odom              -> /automobile/encoder/speed  (Float32 m/s)')
        self.get_logger().info(
            '  /encoder_odom              -> /automobile/encoder/distance (Float32 m)')
        self.get_logger().info(
            '  /automobile/localisation_raw -> /automobile/localisation (String JSON, 1 s delay)')

    # ------------------------------------------------------------------ #
    #  Encoder
    # ------------------------------------------------------------------ #
    def _odom_cb(self, msg: Odometry):
        speed = Float32()
        speed.data = float(msg.twist.twist.linear.x)
        self.pub_speed.publish(speed)

        pos = msg.pose.pose.position
        if self._last_pos is not None:
            dx = pos.x - self._last_pos[0]
            dy = pos.y - self._last_pos[1]
            self._total_dist += math.sqrt(dx * dx + dy * dy)
        self._last_pos = (pos.x, pos.y)

        dist = Float32()
        dist.data = float(self._total_dist)
        self.pub_distance.publish(dist)

    # ------------------------------------------------------------------ #
    #  GPS — buffer incoming poses, publish after GPS_DELAY_S
    # ------------------------------------------------------------------ #
    def _gps_cb(self, msg: Pose):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        self._gps_buf.append((now_s, msg))

    def _flush_gps_buf(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        while self._gps_buf and now_s - self._gps_buf[0][0] >= self.GPS_DELAY_S:
            _, pose = self._gps_buf.popleft()
            quality = int(max(0, min(100, round(random.gauss(92.0, 4.0)))))
            payload = json.dumps({
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'z': float(pose.position.z),
                'quality': quality,
            })
            out = String()
            out.data = payload
            self.pub_gps.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SensorConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
