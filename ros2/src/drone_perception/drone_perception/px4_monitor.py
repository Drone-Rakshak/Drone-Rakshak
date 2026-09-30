#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import String

from px4_msgs.msg import (
    VehicleOdometry,
    VehicleGlobalPosition,
    VehicleAttitude,
    BatteryStatus,
    VehicleStatus,
)


class PX4Monitor(Node):

    def __init__(self):
        super().__init__("px4_monitor")

        # ============================================================
        # PX4 QoS Profile
        # ============================================================
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ============================================================
        # Data storage
        # ============================================================
        self.odom = None
        self.gps = None
        self.att = None
        self.battery = None
        self.status = None

        # Radar / RF data
        self.radar_data = "Waiting for radar data..."
        self.rf_data = "Waiting for antenna/RF data..."

        # ============================================================
        # PX4 subscriptions
        # ============================================================

        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odom_callback,
            qos,
        )

        self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.gps_callback,
            qos,
        )

        self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.att_callback,
            qos,
        )

        self.create_subscription(
            BatteryStatus,
            "/fmu/out/battery_status_v1",
            self.battery_callback,
            qos,
        )

        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.status_callback,
            qos,
        )

        # ============================================================
        # RADAR
        # /drone/radar_detection
        # Type: std_msgs/msg/String
        # ============================================================

        self.create_subscription(
            String,
            "/drone/radar_detection",
            self.radar_callback,
            10,
        )

        # ============================================================
        # ANTENNA / RF
        # /drone/rf_detection
        # Type: std_msgs/msg/String
        # ============================================================

        self.create_subscription(
            String,
            "/drone/rf_detection",
            self.rf_callback,
            10,
        )

        # ============================================================
        # Print monitor every second
        # ============================================================

        self.create_timer(1.0, self.print_status)

        self.get_logger().info("PX4 Monitor Started")
        self.get_logger().info("Radar input: /drone/radar_detection")
        self.get_logger().info("RF input: /drone/rf_detection")

    # ================================================================
    # PX4 callbacks
    # ================================================================

    def odom_callback(self, msg):
        self.odom = msg

    def gps_callback(self, msg):
        self.gps = msg

    def att_callback(self, msg):
        self.att = msg

    def battery_callback(self, msg):
        self.battery = msg

    def status_callback(self, msg):
        self.status = msg

    # ================================================================
    # Radar callback
    # ================================================================

    def radar_callback(self, msg):
        self.radar_data = msg.data

    # ================================================================
    # RF / Antenna callback
    # ================================================================

    def rf_callback(self, msg):
        self.rf_data = msg.data

    # ================================================================
    # Display monitor
    # ================================================================

    def print_status(self):

        # Keep the existing requirement that PX4 data must be available.
        if (
            self.odom is None
            or self.gps is None
            or self.att is None
            or self.battery is None
            or self.status is None
        ):
            return

        # ============================================================
        # Quaternion -> Yaw
        # ============================================================

        q = self.att.q

        yaw = math.atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2),
        )

        # ============================================================
        # Clear terminal
        # ============================================================

        print("\033c", end="")

        print("=" * 70)
        print("                 DRONE RAKSHAK PX4 MONITOR")
        print("=" * 70)

        # ============================================================
        # FLIGHT STATUS
        # ============================================================

        print("\nFLIGHT STATUS")
        print("-" * 70)

        print(f"Navigation State : {self.status.nav_state}")
        print(f"Battery          : {self.battery.remaining * 100:.1f}%")

        # ============================================================
        # GPS
        # ============================================================

        print("\nGPS")
        print("-" * 70)

        print(f"Latitude         : {self.gps.lat:.7f}")
        print(f"Longitude        : {self.gps.lon:.7f}")
        print(f"Altitude         : {self.gps.alt:.2f} m")

        # ============================================================
        # LOCAL POSITION
        # ============================================================

        print("\nLOCAL POSITION")
        print("-" * 70)

        print(f"X                : {self.odom.position[0]:.2f} m")
        print(f"Y                : {self.odom.position[1]:.2f} m")
        print(f"Z                : {self.odom.position[2]:.2f} m")

        # ============================================================
        # VELOCITY
        # ============================================================

        print("\nVELOCITY")
        print("-" * 70)

        print(f"VX               : {self.odom.velocity[0]:.2f} m/s")
        print(f"VY               : {self.odom.velocity[1]:.2f} m/s")
        print(f"VZ               : {self.odom.velocity[2]:.2f} m/s")

        # ============================================================
        # ATTITUDE
        # ============================================================

        print("\nATTITUDE")
        print("-" * 70)

        print(f"Yaw              : {math.degrees(yaw):.2f}°")

        # ============================================================
        # RADAR
        # ============================================================

        print("\nRADAR")
        print("-" * 70)

        print(f"Detection        : {self.radar_data}")

        # ============================================================
        # ANTENNA / RF
        # ============================================================

        print("\nANTENNA / RF")
        print("-" * 70)

        print(f"Detection        : {self.rf_data}")

        # ============================================================
        # Footer
        # ============================================================

        print("\n" + "=" * 70)
        print("Radar + RF data integrated with PX4 monitoring")
        print("=" * 70)


def main(args=None):

    rclpy.init(args=args)

    node = PX4Monitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
