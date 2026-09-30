#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class AxisCheck(Node):

    def __init__(self):
        super().__init__('axis_check')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.callback,
            10
        )

        self.done = False

    def callback(self, msg):

        if self.done:
            return

        points = point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        )

        valid = []

        for p in points:

            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            if (
                math.isfinite(x)
                and math.isfinite(y)
                and math.isfinite(z)
            ):
                valid.append((x, y, z))

        if not valid:
            self.get_logger().warn(
                'No valid points received'
            )
            return

        xs = [p[0] for p in valid]
        ys = [p[1] for p in valid]
        zs = [p[2] for p in valid]

        self.get_logger().info(
            f'Valid points: {len(valid)}'
        )

        self.get_logger().info(
            f'X range: {min(xs):.3f} to {max(xs):.3f}'
        )

        self.get_logger().info(
            f'Y range: {min(ys):.3f} to {max(ys):.3f}'
        )

        self.get_logger().info(
            f'Z range: {min(zs):.3f} to {max(zs):.3f}'
        )

        self.get_logger().info(
            'First 20 valid points:'
        )

        for p in valid[:20]:

            self.get_logger().info(
                f'X={p[0]:.3f}, '
                f'Y={p[1]:.3f}, '
                f'Z={p[2]:.3f}'
            )

        self.done = True


def main():

    rclpy.init()

    node = AxisCheck()

    rclpy.spin_once(
        node,
        timeout_sec=10.0
    )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
