import time

from px4_msgs.msg import (
    VehicleGlobalPosition,
    VehicleAttitude,
    BatteryStatus,
    VehicleStatus,
)

from bridge.config import UAV_ID
from bridge.services.telemetry_manager import TelemetryManager


class PX4TelemetrySubscriber:

    def __init__(self, node):

        self.node = node

        self.manager = TelemetryManager()

        self.latitude = None
        self.longitude = None
        self.altitude = None

        self.roll = None
        self.pitch = None
        self.yaw = None

        self.battery = None

        self.armed = None
        self.flight_mode = None

        self.last_publish_time = 0

        node.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.global_position_callback,
            10,
        )

        node.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.attitude_callback,
            10,
        )

        node.create_subscription(
            BatteryStatus,
            "/fmu/out/battery_status_v1",
            self.battery_callback,
            10,
        )

        node.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.status_callback,
            10,
        )

        node.get_logger().info(
            "PX4 telemetry subscribers initialized"
        )

    # --------------------------------------------------
    # GLOBAL POSITION
    # --------------------------------------------------

    def global_position_callback(self, msg):

        if msg.lat_lon_valid:

            self.latitude = msg.lat
            self.longitude = msg.lon

        if msg.alt_valid:

            self.altitude = msg.alt

    # --------------------------------------------------
    # ATTITUDE
    # --------------------------------------------------

    def attitude_callback(self, msg):

        (
            self.roll,
            self.pitch,
            self.yaw,
        ) = self.manager.quaternion_to_euler(
            msg.q
        )

    # --------------------------------------------------
    # BATTERY
    # --------------------------------------------------

    def battery_callback(self, msg):

        if msg.connected:

            if msg.remaining >= 0:

                self.battery = (
                    msg.remaining * 100.0
                )

    # --------------------------------------------------
    # VEHICLE STATUS
    # --------------------------------------------------

    def status_callback(self, msg):

        self.armed = (
            msg.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )

        self.flight_mode = (
            self.get_navigation_mode(
                msg.nav_state
            )
        )

    # --------------------------------------------------
    # NAVIGATION MODE
    # --------------------------------------------------

    def get_navigation_mode(
        self,
        nav_state,
    ):

        modes = {

            VehicleStatus.NAVIGATION_STATE_MANUAL:
                "MANUAL",

            VehicleStatus.NAVIGATION_STATE_ALTCTL:
                "ALTCTL",

            VehicleStatus.NAVIGATION_STATE_POSCTL:
                "POSCTL",

            VehicleStatus.NAVIGATION_STATE_AUTO_MISSION:
                "AUTO_MISSION",

            VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                "AUTO_LOITER",

            VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
                "AUTO_RTL",

            VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                "OFFBOARD",

            VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                "AUTO_TAKEOFF",

            VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                "AUTO_LAND",

        }

        return modes.get(
            nav_state,
            f"UNKNOWN_{nav_state}",
        )

    # --------------------------------------------------
    # BUILD TELEMETRY
    # --------------------------------------------------

    def build_telemetry(self):

        return {

            "uav_id": UAV_ID,

            "timestamp": time.time(),

            "position": {

                "latitude":
                    self.latitude,

                "longitude":
                    self.longitude,

                "altitude":
                    self.altitude,
            },

            "attitude": {

                "roll":
                    self.roll,

                "pitch":
                    self.pitch,

                "yaw":
                    self.yaw,
            },

            "velocity": {},

            "battery": {

                "percentage":
                    self.battery,

            },

            "gnss": {},

            "armed":
                self.armed,

            "flight_mode":
                self.flight_mode,
        }
