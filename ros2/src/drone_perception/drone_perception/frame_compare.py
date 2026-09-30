#!/usr/bin/env python3

import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge


class FrameCompare(Node):

    def __init__(self):

        super().__init__("frame_compare")

        self.bridge = CvBridge()

        self.previous = None

        self.subscription = self.create_subscription(
            Image,
            "/drone/processed_image",
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            "/drone/motion_image",
            10
        )

        self.get_logger().info("Frame Compare Started")


    def callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="mono8"
        )

        if self.previous is None:
            self.previous = frame
            return

        diff = cv2.absdiff(self.previous, frame)

        _, thresh = cv2.threshold(
            diff,
            25,
            255,
            cv2.THRESH_BINARY
        )

        self.previous = frame

        output = self.bridge.cv2_to_imgmsg(
            thresh,
            encoding="mono8"
        )

        self.publisher.publish(output)


def main(args=None):

    rclpy.init(args=args)

    node = FrameCompare()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
