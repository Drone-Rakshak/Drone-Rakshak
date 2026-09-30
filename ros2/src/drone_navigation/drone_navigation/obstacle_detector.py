#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Float32MultiArray


class ObstacleDetector(Node):

    def __init__(self):

        super().__init__("obstacle_detector")

        # ============================================================
        # CONFIGURATION
        # ============================================================

        self.max_range = 30.0
        self.obstacle_threshold = 2.0

        # Used to report only changes in FRONT obstacle state
        self.front_obstacle = False

        # ============================================================
        # LIDAR SUBSCRIPTION
        # ============================================================

        self.lidar_sub = self.create_subscription(
            LaserScan,
            "/world/drone_rakshak/model/x500_0/link/link/sensor/lidar_2d_v2/scan",
            self.lidar_callback,
            10
        )

        # ============================================================
        # OUTPUT: CLEARANCE
        #
        # [Forward, Backward, Left, Right, Up, Down]
        # ============================================================

        self.clearance_pub = self.create_publisher(
            Float32MultiArray,
            "/obstacle_clearance",
            10
        )

        # ============================================================
        # OUTPUT: FORWARD DISTANCE
        # ============================================================

        self.distance_pub = self.create_publisher(
            Float32,
            "/obstacle_distance",
            10
        )

    # ================================================================
    # LIDAR CALLBACK
    # ================================================================

    def lidar_callback(self, msg):

        ranges = np.array(
            msg.ranges,
            dtype=np.float32
        )

        angles = (
            msg.angle_min
            + np.arange(len(ranges))
            * msg.angle_increment
        )

        # ------------------------------------------------------------
        # Valid measurements
        # ------------------------------------------------------------

        valid = (
            np.isfinite(ranges)
            & (ranges >= msg.range_min)
            & (ranges <= msg.range_max)
        )

        ranges = ranges[valid]
        angles = angles[valid]

        # ------------------------------------------------------------
        # No valid LiDAR data
        # ------------------------------------------------------------

        if len(ranges) == 0:

            self.publish_clearance(
                self.max_range,
                self.max_range,
                self.max_range,
                self.max_range,
                self.max_range,
                self.max_range
            )

            self.publish_distance(
                self.max_range
            )

            return

        # ------------------------------------------------------------
        # Angular sectors
        # ------------------------------------------------------------

        sector_half_width = math.radians(20)

        # FRONT
        forward_mask = (
            np.abs(angles)
            <= sector_half_width
        )

        # LEFT
        left_mask = (
            (angles >= math.radians(70))
            & (angles <= math.radians(110))
        )

        # RIGHT
        right_mask = (
            (angles >= math.radians(-110))
            & (angles <= math.radians(-70))
        )

        # BACK
        backward_mask = (
            np.abs(angles)
            >= math.radians(160)
        )

        # ------------------------------------------------------------
        # Calculate clearance
        # ------------------------------------------------------------

        forward_distance = self.get_sector_min(
            ranges,
            forward_mask
        )

        backward_distance = self.get_sector_min(
            ranges,
            backward_mask
        )

        left_distance = self.get_sector_min(
            ranges,
            left_mask
        )

        right_distance = self.get_sector_min(
            ranges,
            right_mask
        )

        # ------------------------------------------------------------
        # 2D LiDAR cannot measure vertical clearance
        # ------------------------------------------------------------

        up_distance = self.max_range
        down_distance = self.max_range

        # ------------------------------------------------------------
        # Publish continuous backend data
        # ------------------------------------------------------------

        self.publish_clearance(
            forward_distance,
            backward_distance,
            left_distance,
            right_distance,
            up_distance,
            down_distance
        )

        self.publish_distance(
            forward_distance
        )

        # ============================================================
        # FRONT OBSTACLE EVENT ONLY
        # ============================================================

        obstacle_now = (
            forward_distance
            < self.obstacle_threshold
        )

        # Only print when state changes
        if obstacle_now and not self.front_obstacle:

            self.get_logger().warn(
                f"FRONT OBSTACLE DETECTED | "
                f"Distance: {forward_distance:.2f} m"
            )

        elif (
            not obstacle_now
            and self.front_obstacle
        ):

            self.get_logger().info(
                "FRONT PATH CLEAR"
            )

        self.front_obstacle = obstacle_now

    # ================================================================
    # SECTOR MINIMUM
    # ================================================================

    def get_sector_min(
        self,
        ranges,
        mask
    ):

        if np.any(mask):

            return float(
                min(
                    np.min(ranges[mask]),
                    self.max_range
                )
            )

        return self.max_range

    # ================================================================
    # CLEARANCE PUBLISHER
    # ================================================================

    def publish_clearance(
        self,
        forward,
        backward,
        left,
        right,
        up,
        down
    ):

        msg = Float32MultiArray()

        msg.data = [
            float(forward),
            float(backward),
            float(left),
            float(right),
            float(up),
            float(down)
        ]

        self.clearance_pub.publish(msg)

    # ================================================================
    # DISTANCE PUBLISHER
    # ================================================================

    def publish_distance(
        self,
        distance
    ):

        msg = Float32()

        msg.data = float(distance)

        self.distance_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
