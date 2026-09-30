#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image


class ImageBuffer(Node):

    def __init__(self):

        super().__init__("image_buffer")

        self.latest_image = None

        self.frame_count = 0

        self.create_subscription(
            Image,
            "/world/drone_rakshak/model/x500_0/link/camera_link/sensor/camera/image",
            self.image_callback,
            10,
        )

        self.create_timer(3.0, self.print_status)

        self.get_logger().info("Image Buffer Started")

    def image_callback(self, msg):

        self.latest_image = msg

        self.frame_count += 1

    def print_status(self):

        if self.latest_image is None:

            self.get_logger().info("Waiting for camera...")

            return

        self.get_logger().info(
            f"""
================ IMAGE BUFFER ================

Frames Received : {self.frame_count}

Image Size      : {self.latest_image.width} x {self.latest_image.height}

Encoding        : {self.latest_image.encoding}

==============================================
"""
        )


def main(args=None):

    rclpy.init(args=args)

    node = ImageBuffer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()
