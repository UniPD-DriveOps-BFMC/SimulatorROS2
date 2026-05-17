#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GpsConverterNode(Node):
    """
    Converts /automobile/localisation (Pose, world x/y in metres) to
    /automobile/gps (sensor_msgs/NavSatFix) using a configurable map origin.

    Coordinate convention (Gazebo world frame):
      x → East  (increases longitude)
      y → North (increases latitude)
    """

    # Metres per degree of latitude (constant)
    _M_PER_DEG_LAT = 111_139.0

    def __init__(self):
        super().__init__('gps_converter')

        self.declare_parameter('origin_lat', 46.7712)   # Cluj-Napoca, Romania
        self.declare_parameter('origin_lon', 23.6236)
        self.declare_parameter('origin_alt', 340.0)     # metres above sea level

        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        self.origin_alt = self.get_parameter('origin_alt').value

        self._m_per_deg_lon = (
            self._M_PER_DEG_LAT * math.cos(math.radians(self.origin_lat))
        )

        self.sub = self.create_subscription(
            Pose, '/automobile/localisation', self.callback, 10)
        self.pub = self.create_publisher(NavSatFix, '/automobile/gps', 10)

        self.get_logger().info(
            f'GPS converter started. Origin: {self.origin_lat:.6f}°N, '
            f'{self.origin_lon:.6f}°E, alt={self.origin_alt:.1f} m'
        )

    def callback(self, msg: Pose):
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps'

        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS

        fix.latitude  = self.origin_lat + msg.position.y / self._M_PER_DEG_LAT
        fix.longitude = self.origin_lon + msg.position.x / self._m_per_deg_lon
        fix.altitude  = self.origin_alt + msg.position.z

        # Covariance: ±0.1 m GPS noise → variance ≈ 0.01 m²
        cov = 0.01
        fix.position_covariance = [
            cov, 0.0, 0.0,
            0.0, cov, 0.0,
            0.0, 0.0, cov * 4.0,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = GpsConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
