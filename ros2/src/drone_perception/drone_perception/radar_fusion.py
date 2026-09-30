#!/usr/bin/env python3

import re

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


class RadarFusion(Node):

    def __init__(self):

        super().__init__("radar_fusion")

        # ============================================================
        # CONFIGURATION
        # ============================================================

        self.DEFAULT_CLEARANCE = 30.0

        # Latest LiDAR clearance
        #
        # [F, B, L, R, U, D]
        self.lidar_clearance = None

        # Latest radar obstacle
        self.radar_distance = None
        self.radar_bearing = None

        # ============================================================
        # SUBSCRIPTIONS
        # ============================================================

        self.lidar_sub = self.create_subscription(
            Float32MultiArray,
            "/obstacle_clearance",
            self.lidar_callback,
            10
        )

        self.radar_sub = self.create_subscription(
            String,
            "/drone/radar_obstacle",
            self.radar_callback,
            10
        )

        # ============================================================
        # OUTPUT
        # ============================================================

        self.fused_pub = self.create_publisher(
            Float32MultiArray,
            "/drone/fused_clearance",
            10
        )

        # Publish fusion data continuously at 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.publish_fused_data
        )

    # ============================================================
    # LIDAR
    # ============================================================

    def lidar_callback(self, msg):

        if len(msg.data) < 6:
            return

        self.lidar_clearance = [
            float(x)
            for x in msg.data[:6]
        ]

    # ============================================================
    # RADAR
    # ============================================================

    def radar_callback(self, msg):

        text = msg.data

        distance_match = re.search(
            r"Distance=([0-9.]+)\s*m",
            text
        )

        bearing_match = re.search(
            r"Bearing=([0-9.]+)\s*deg",
            text
        )

        if (
            distance_match is None
            or bearing_match is None
        ):
            return

        self.radar_distance = float(
            distance_match.group(1)
        )

        self.radar_bearing = float(
            bearing_match.group(1)
        )

    # ============================================================
    # FUSION
    # ============================================================

    def publish_fused_data(self):

        # --------------------------------------------------------
        # Start with LiDAR
        # --------------------------------------------------------

        if self.lidar_clearance is not None:

            fused = list(
                self.lidar_clearance
            )

        else:

            fused = [
                self.DEFAULT_CLEARANCE,
                self.DEFAULT_CLEARANCE,
                self.DEFAULT_CLEARANCE,
                self.DEFAULT_CLEARANCE,
                self.DEFAULT_CLEARANCE,
                self.DEFAULT_CLEARANCE
            ]

        # --------------------------------------------------------
        # Fuse Radar data
        # --------------------------------------------------------

        if (
            self.radar_distance is not None
            and self.radar_bearing is not None
        ):

            # Limit radar contribution to 30 meters
            distance = min(
                self.radar_distance,
                self.DEFAULT_CLEARANCE
            )

            bearing = (
                self.radar_bearing % 360.0
            )

            # ----------------------------------------------------
            # Forward
            # 315° -> 360° and 0° -> 45°
            # ----------------------------------------------------

            if (
                bearing >= 315.0
                or bearing <= 45.0
            ):

                fused[0] = min(
                    fused[0],
                    distance
                )

            # ----------------------------------------------------
            # Right
            # 45° -> 135°
            # ----------------------------------------------------

            elif (
                bearing > 45.0
                and bearing <= 135.0
            ):

                fused[3] = min(
                    fused[3],
                    distance
                )

            # ----------------------------------------------------
            # Backward
            # 135° -> 225°
            # ----------------------------------------------------

            elif (
                bearing > 135.0
                and bearing <= 225.0
            ):

                fused[1] = min(
                    fused[1],
                    distance
                )

            # ----------------------------------------------------
            # Left
            # 225° -> 315°
            # ----------------------------------------------------

            else:

                fused[2] = min(
                    fused[2],
                    distance
                )

        # --------------------------------------------------------
        # Publish continuously
        # --------------------------------------------------------

        output = Float32MultiArray()

        output.data = [
            float(value)
            for value in fused
        ]

        self.fused_pub.publish(
            output
        )


# ================================================================
# MAIN
# ================================================================

def main(args=None):

    rclpy.init(args=args)

    node = RadarFusion()

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
