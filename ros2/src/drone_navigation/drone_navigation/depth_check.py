import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import math


class DepthChecker(Node):

    def __init__(self):
        super().__init__('depth_checker')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.callback,
            10
        )

        self.received = False

    def callback(self, msg):

        self.received = True

        points = point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=False
        )

        total = 0
        valid = 0
        invalid = 0

        min_z = float('inf')
        max_z = float('-inf')

        for point in points:

            x = float(point['x'])
            y = float(point['y'])
            z = float(point['z'])

            total += 1

            if (
                math.isfinite(x)
                and math.isfinite(y)
                and math.isfinite(z)
            ):
                valid += 1

                min_z = min(min_z, z)
                max_z = max(max_z, z)

            else:
                invalid += 1

        if valid > 0:

            self.get_logger().info(
                f'PointCloud: {msg.width}x{msg.height} | '
                f'Total={total} | '
                f'Valid={valid} | '
                f'Invalid={invalid} | '
                f'Z range={min_z:.2f} to {max_z:.2f} m'
            )

        else:

            self.get_logger().warn(
                f'NO VALID POINTS | '
                f'Total={total} | '
                f'Invalid={invalid}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = DepthChecker()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
