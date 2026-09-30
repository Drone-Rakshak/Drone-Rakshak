#!/usr/bin/env python3

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node

from sensor_msgs.msg import Image


class MotionDetector(Node):

    def __init__(self):

        super().__init__("motion_detector")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            "/drone/motion_image",
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            "/drone/detection_image",
            10
        )

        self.get_logger().info("Motion Detector Started")

    def callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="mono8"
        )

        contours, _ = cv2.findContours(
            image,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        output = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        for contour in contours:

            area = cv2.contourArea(contour)

            if area < 50:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            cv2.rectangle(
                output,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

        self.publisher.publish(
            self.bridge.cv2_to_imgmsg(
                output,
                encoding="bgr8"
            )
        )


def main(args=None):

    rclpy.init(args=args)

    node = MotionDetector()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
