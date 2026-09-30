import rclpy
import random

from rclpy.node import Node

from std_msgs.msg import String


class RFReceiver(Node):

    def __init__(self):

        super().__init__("rf_receiver")

        self.publisher = self.create_publisher(
            String,
            "/drone/rf_detection",
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_detection
        )

        self.get_logger().info("RF Receiver Started")

    def publish_detection(self):

        protocols = [
            "Drone Telemetry",
            "WiFi",
            "Unknown RF",
            "LoRa",
            "LTE",
            "Control Link"
        ]

        protocol = random.choice(protocols)

        strength = random.randint(-90, -40)

        bearing = random.randint(0, 359)

        distance = random.randint(50, 500)

        confidence = round(random.uniform(0.60, 0.99), 2)

        if strength > -60:
            threat = "HIGH"
        elif strength > -75:
            threat = "MEDIUM"
        else:
            threat = "LOW"

        msg = String()

        msg.data = (
            f"Protocol={protocol}, "
            f"Strength={strength} dBm, "
            f"Bearing={bearing} deg, "
            f"Distance={distance} m, "
            f"Confidence={confidence}, "
            f"Threat={threat}"
        )

        self.publisher.publish(msg)

        self.get_logger().info(msg.data)


def main():

    rclpy.init()

    node = RFReceiver()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
