import rclpy
from rclpy.node import Node

from bridge.ros2.subscribers import (
    PX4TelemetrySubscriber,
)
from bridge.transport.backend_client import (
    BackendClient,
)


class DroneRakshakNode(Node):

    def __init__(self):

        super().__init__(
            "drone_rakshak_telemetry_bridge"
        )

        self.backend = BackendClient()

        self.telemetry = (
            PX4TelemetrySubscriber(
                self
            )
        )

        # Publish telemetry to backend
        # every 500 ms = 2 Hz

        self.timer = self.create_timer(
            0.5,
            self.publish_telemetry,
        )

        self.get_logger().info(
            "Drone Rakshak Telemetry Bridge started"
        )

    def publish_telemetry(self):

        data = (
            self.telemetry.build_telemetry()
        )

        success = (
            self.backend.send_telemetry(
                data
            )
        )

        if success:

            self.get_logger().info(
                "Telemetry sent to backend"
            )
