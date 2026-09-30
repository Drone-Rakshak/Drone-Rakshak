import rclpy

from bridge.ros2.node import (
    DroneRakshakNode,
)


def main(args=None):

    rclpy.init(args=args)

    node = DroneRakshakNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()
