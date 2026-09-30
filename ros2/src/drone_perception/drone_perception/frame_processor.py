#!/usr/bin/env python3

import cv2

from cv_bridge import CvBridge

from rclpy.node import Node
import rclpy

from sensor_msgs.msg import Image


class FrameProcessor(Node):

    def __init__(self):

        super().__init__("frame_processor")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            "/world/drone_rakshak/model/x500_0/link/camera_link/sensor/camera/image",
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            "/drone/processed_image",
            10
        )

        self.get_logger().info("Frame Processor Started")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        # Resize
        frame = cv2.resize(frame, (640, 480))

        # Blur
        frame = cv2.GaussianBlur(frame, (5, 5), 0)

        # Convert to Gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        processed = self.bridge.cv2_to_imgmsg(
            gray,
            encoding="mono8"
        )

        self.publisher.publish(processed)


def main(args=None):

    rclpy.init(args=args)

    node = FrameProcessor()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
