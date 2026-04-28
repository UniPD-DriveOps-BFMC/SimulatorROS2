#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np


class MonoConverterNode(Node):
    """
    1. Converte RGB -> mono8 per le camere left/right (OV9282 simulato)
    2. Corregge il frame_id della depth image per allinearlo alla RGB
       (richiesto da depth_image_proc per generare PointCloud2)
    """

    def __init__(self):
        super().__init__('oak_mono_converter')

        # --- Mono LEFT ---
        self.sub_left = self.create_subscription(
            Image, '/oak/left/image_raw', self.callback_left, 10)
        self.pub_left = self.create_publisher(
            Image, '/oak/left/image_mono', 10)

        # --- Mono RIGHT ---
        self.sub_right = self.create_subscription(
            Image, '/oak/right/image_raw', self.callback_right, 10)
        self.pub_right = self.create_publisher(
            Image, '/oak/right/image_mono', 10)

        # --- Depth frame_id fix ---
        # Riscrive il frame_id della depth da oak_depth -> oak_rgb
        # così depth_image_proc non genera il warning e la PointCloud è corretta
        self.sub_depth = self.create_subscription(
            Image, '/oak/stereo/image_raw', self.callback_depth, 10)
        self.pub_depth = self.create_publisher(
            Image, '/oak/stereo/image_fixed', 10)

        self.get_logger().info('OAK-D Mono Converter + Depth Fix avviato.')
        self.get_logger().info('  /oak/left/image_raw    -> /oak/left/image_mono')
        self.get_logger().info('  /oak/right/image_raw   -> /oak/right/image_mono')
        self.get_logger().info('  /oak/stereo/image_raw  -> /oak/stereo/image_fixed (frame_id corretto)')

    def rgb_to_mono(self, msg: Image) -> Image:
        """
        Converte rgb8 -> mono8 usando formula luminanza ITU-R BT.601:
        Y = 0.299*R + 0.587*G + 0.114*B
        """
        np_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)

        gray = (
            0.299 * np_img[:, :, 0].astype(np.float32) +
            0.587 * np_img[:, :, 1].astype(np.float32) +
            0.114 * np_img[:, :, 2].astype(np.float32)
        ).astype(np.uint8)

        mono_msg = Image()
        mono_msg.header = msg.header
        mono_msg.height = msg.height
        mono_msg.width = msg.width
        mono_msg.encoding = 'mono8'
        mono_msg.is_bigendian = 0
        mono_msg.step = msg.width
        mono_msg.data = gray.tobytes()
        return mono_msg

    def callback_left(self, msg: Image):
        self.pub_left.publish(self.rgb_to_mono(msg))

    def callback_right(self, msg: Image):
        self.pub_right.publish(self.rgb_to_mono(msg))

    def callback_depth(self, msg: Image):
        """Riscrive solo il frame_id, lascia i dati depth invariati."""
        fixed = Image()
        fixed.header = msg.header
        fixed.header.frame_id = 'automobile/camera/link_camera/oak_rgb'
        fixed.height = msg.height
        fixed.width = msg.width
        fixed.encoding = msg.encoding
        fixed.is_bigendian = msg.is_bigendian
        fixed.step = msg.step
        fixed.data = msg.data
        self.pub_depth.publish(fixed)

    def main(args=None):
        rclpy.init(args=args)
        node = MonoConverterNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MonoConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
