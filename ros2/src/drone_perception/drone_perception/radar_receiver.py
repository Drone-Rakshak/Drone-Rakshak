#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RadarReceiver(Node):

    def __init__(self):

        super().__init__("radar_receiver")

        # ============================================================
        # RADAR OUTPUT TOPICS
        # ============================================================

        self.detection_pub = self.create_publisher(
            String,
            "/drone/radar_detection",
            10
        )

        self.threat_pub = self.create_publisher(
            String,
            "/drone/radar_threat",
            10
        )

        self.obstacle_pub = self.create_publisher(
            String,
            "/drone/radar_obstacle",
            10
        )

        # ============================================================
        # RADAR UPDATE
        # ============================================================

        self.timer = self.create_timer(
            0.5,
            self.publish_detection
        )

        # ============================================================
        # IMPORTANT:
        # NO LOGGER OUTPUT HERE
        #
        # Radar works completely in backend.
        # ============================================================

    # ================================================================
    # RADAR SIMULATION
    # ================================================================

    def publish_detection(self):

        classes = [
            "Drone",
            "Bird",
            "Helicopter",
            "Vehicle",
            "Unknown"
        ]

        obj = random.choice(classes)

        distance = random.randint(
            5,
            300
        )

        bearing = random.randint(
            0,
            359
        )

        velocity = round(
            random.uniform(-25, 25),
            2
        )

        rcs = round(
            random.uniform(0.2, 5.0),
            2
        )

        confidence = round(
            random.uniform(0.60, 0.99),
            2
        )

        # ============================================================
        # THREAT CLASSIFICATION
        # ============================================================

        if obj in [
            "Drone",
            "Helicopter"
        ]:

            threat = "HIGH"

        elif obj == "Vehicle":

            threat = "MEDIUM"

        elif obj == "Bird":

            threat = "LOW"

        else:

            threat = "UNKNOWN"

        # ============================================================
        # GENERAL RADAR DATA
        #
        # Published continuously.
        # NOT printed.
        # ============================================================

        radar_text = (
            f"Object={obj}, "
            f"Distance={distance} m, "
            f"Bearing={bearing} deg, "
            f"Velocity={velocity} m/s, "
            f"RCS={rcs}, "
            f"Confidence={confidence}, "
            f"Threat={threat}"
        )

        detection_msg = String()

        detection_msg.data = radar_text

        self.detection_pub.publish(
            detection_msg
        )

        # ============================================================
        # AERIAL TARGET
        #
        # Published to /drone/radar_threat.
        # NOT printed.
        # ============================================================

        if obj in [
            "Drone",
            "Bird",
            "Helicopter"
        ]:

            threat_msg = String()

            threat_msg.data = (
                f"AERIAL TARGET | "
                f"Object={obj} | "
                f"Distance={distance} m | "
                f"Bearing={bearing} deg | "
                f"Velocity={velocity} m/s | "
                f"Threat={threat}"
            )

            self.threat_pub.publish(
                threat_msg
            )

        # ============================================================
        # RADAR OBSTACLE
        #
        # Published to /drone/radar_obstacle.
        # NOT printed.
        # ============================================================

        if distance <= 30:

            obstacle_msg = String()

            obstacle_msg.data = (
                f"RADAR OBSTACLE | "
                f"Object={obj} | "
                f"Distance={distance} m | "
                f"Bearing={bearing} deg"
            )

            self.obstacle_pub.publish(
                obstacle_msg
            )


def main(args=None):

    rclpy.init(args=args)

    node = RadarReceiver()

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
