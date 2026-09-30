#!/usr/bin/env python3

import math

import rclpy
import time
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from std_msgs.msg import Float32MultiArray, String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleStatus,
)


class OffboardController(Node):

    def __init__(self):
        super().__init__("offboard_controller")

        self.get_logger().info("")
        self.get_logger().info("======================================")
        self.get_logger().info(" Drone Rakshak Dynamic 3D Controller")
        self.get_logger().info(" Bidirectional Command Interface Enabled")
        self.get_logger().info("======================================")

        # ============================================================
        # QoS
        # ============================================================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Dedicated QoS for the Offboard heartbeat.
        # Keep it volatile because PX4's inbound subscription is volatile.
        offboard_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ============================================================
        # PX4 PUBLISHERS
        # ============================================================

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            offboard_qos,
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            px4_qos,
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            px4_qos,
        )

        # ============================================================
        # PX4 SUBSCRIBERS
        # ============================================================

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            px4_qos,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.status_callback,
            px4_qos,
        )

        self.ack_sub = self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack_v1",
            self.ack_callback,
            px4_qos,
        )

        # ============================================================
        # 3D OBSTACLE CLEARANCE
        #
        # [forward, backward, left, right, up, down]
        # ============================================================

        self.clearance_sub = self.create_subscription(
            Float32MultiArray,
            "/drone/fused_clearance",
            self.clearance_callback,
            sensor_qos,
        )

        # ============================================================
        # BACKEND AVOIDANCE DECISION
        # ============================================================

        self.avoidance_decision_sub = self.create_subscription(
            String,
            "/drone/avoidance_decision",
            self.avoidance_decision_callback,
            10,
        )

        # ============================================================
        # BIDIRECTIONAL COMMAND INPUT
        # ============================================================

        self.command_sub = self.create_subscription(
            String,
            "/drone_command",
            self.command_callback,
            10,
        )

        # Command feedback
        self.command_status_pub = self.create_publisher(
            String,
            "/drone_command_status",
            10,
        )

        # ============================================================
        # VEHICLE STATE
        # ============================================================

        self.current_position = None
        self.vehicle_status = None

        self.home_x = None
        self.home_y = None

        # ============================================================
        # OBSTACLE STATE
        # ============================================================

        self.clearance = [
            8.0,    # forward
            -1.0,   # backward unknown
            8.0,    # left
            8.0,    # right
            8.0,    # up
            8.0,    # down
        ]

        self.clearance_received = False
        self.last_obstacle_time = None

        # Latest decision generated by the backend
        # using fused LiDAR + Radar data.
        self.avoidance_decision = "PATH_CLEAR"

        # ============================================================
        # SAFETY PARAMETERS
        # ============================================================

        self.obstacle_threshold = 1.5

        self.minimum_escape_clearance = 1.2

        self.safety_margin = 0.8

        self.max_avoidance_step = 4.0

        self.minimum_avoidance_step = 0.8

        self.clear_distance = 1.2

        self.required_clear_cycles = 8

        self.sensor_timeout = 0.6

        # ============================================================
        # MISSION WAYPOINTS
        #
        # PX4 NED:
        # X = North
        # Y = East
        # Z = Down
        # ============================================================

        self.waypoints = [
            (73.96, -145.0, -5.0),    # WP0 / base
            (155.0, -115.0, -5.0),    # WP1
            (135.0, -25.0, -5.0),     # WP2
            (45.0, -55.0, -5.0),      # WP3
            (25.0, -175.0, -15.0),    # WP4
        ]

        self.current_waypoint = 0

        # ============================================================
        # DYNAMIC AVOIDANCE STATE
        # ============================================================

        self.avoidance_active = False
        self.avoidance_target = None
        self.avoidance_direction = None
        self.original_target = None

        self.obstacle_clear_counter = 0

        # ============================================================
        # WAYPOINT HANDLING
        # ============================================================

        self.position_tolerance = 0.8

        self.waypoint_hold_cycles_required = 10
        self.waypoint_hold_cycles = 0

        self.return_home_hold_cycles = 0

        # ============================================================
        # EXTERNAL COMMAND STATE
        # ============================================================

        self.command_active = False
        self.command_target = None
        self.command_text = None

        # Used for TURN commands
        self.command_yaw = None
        self.yaw_command_active = False

        # ============================================================
        # STATE MACHINE
        # ============================================================

        self.state = "HEARTBEAT"

        self.heartbeat_count = 0

        self.arm_command_sent = False
        self.offboard_command_sent = False
        self.land_command_sent = False

        # ============================================================
        # 10 Hz CONTROLLER
        # ============================================================

        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

        self.get_logger().info(
            "Dynamic 3D obstacle avoidance enabled"
        )

        self.get_logger().info(
            "Command topic: /drone_command"
        )

        self.get_logger().info(
            "Command status topic: /drone_command_status"
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def position_callback(self, msg):

        self.current_position = msg

        if self.home_x is None and msg.xy_valid:

            self.home_x = float(msg.x)
            self.home_y = float(msg.y)

            self.get_logger().info(
                f"Home position captured: "
                f"({self.home_x:.2f}, {self.home_y:.2f})"
            )

    def status_callback(self, msg):

        self.vehicle_status = msg

    def ack_callback(self, msg):

        self.get_logger().info(
            f"PX4 ACK: command={msg.command}, "
            f"result={msg.result}"
        )

    def clearance_callback(self, msg):

        if len(msg.data) < 6:
            return

        self.clearance = [
            float(msg.data[0]),
            float(msg.data[1]),
            float(msg.data[2]),
            float(msg.data[3]),
            float(msg.data[4]),
            float(msg.data[5]),
        ]

        self.clearance_received = True

        self.last_obstacle_time = self.get_clock().now()

    # ================================================================
    # BACKEND AVOIDANCE DECISION CALLBACK
    # ================================================================

    def avoidance_decision_callback(self, msg):

        self.avoidance_decision = msg.data.strip().upper()

    # ================================================================
    # COMMAND STATUS
    # ================================================================

    def publish_command_status(self, text):

        msg = String()
        msg.data = text

        self.command_status_pub.publish(msg)

        self.get_logger().info(
            f"COMMAND STATUS: {text}"
        )

    # ================================================================
    # BIDIRECTIONAL COMMAND CALLBACK
    # ================================================================

    def command_callback(self, msg):

        command = msg.data.strip()

        if not command:
            return

        self.get_logger().info(
            f"COMMAND RECEIVED: {command}"
        )

        parts = command.upper().split()

        if len(parts) == 0:
            return

        # ============================================================
        # NORMALIZE NATURAL COMMANDS
        # ============================================================

        # Remove accidental copied terminal prompt.
        # Example:
        # COMMAND > turn right 90  -> turn right 90
        if len(parts) >= 2 and parts[0] == "COMMAND" and parts[1] == ">":
            parts = parts[2:]

        if len(parts) == 0:
            return

        # Optional words that make natural commands easier to type.
        # turn right 90 degree -> turn right 90
        if len(parts) >= 1 and parts[-1] in ["DEGREE", "DEGREES"]:
            parts = parts[:-1]

        # MOVE FORWARD 10 -> FORWARD 10
        if len(parts) >= 2 and parts[0] == "MOVE":
            parts = parts[1:]

        # GO TO X Y Z -> GOTO X Y Z
        if len(parts) >= 2 and parts[0] == "GO" and parts[1] == "TO":
            parts = ["GOTO"] + parts[2:]

        # TURN RIGHT 90 -> TURN_RIGHT 90
        if len(parts) >= 2 and parts[0] == "TURN":

            if parts[1] == "RIGHT":
                parts = ["TURN_RIGHT"] + parts[2:]

            elif parts[1] == "LEFT":
                parts = ["TURN_LEFT"] + parts[2:]

        # RETURN HOME -> RETURN_HOME
        if len(parts) >= 2 and parts[0] == "RETURN" and parts[1] == "HOME":
            parts = ["RETURN_HOME"] + parts[2:]

        # Compact diagonal forms.
        # southwest 20 -> south west 20
        # north-east 20 -> north east 20
        compact_diagonals = {
            "NORTHEAST": ["NORTH", "EAST"],
            "NORTH_EAST": ["NORTH", "EAST"],
            "NORTH-EAST": ["NORTH", "EAST"],
            "NORTHWEST": ["NORTH", "WEST"],
            "NORTH_WEST": ["NORTH", "WEST"],
            "NORTH-WEST": ["NORTH", "WEST"],
            "SOUTHEAST": ["SOUTH", "EAST"],
            "SOUTH_EAST": ["SOUTH", "EAST"],
            "SOUTH-EAST": ["SOUTH", "EAST"],
            "SOUTHWEST": ["SOUTH", "WEST"],
            "SOUTH_WEST": ["SOUTH", "WEST"],
            "SOUTH-WEST": ["SOUTH", "WEST"],
        }

        if len(parts) >= 1 and parts[0] in compact_diagonals:
            parts = compact_diagonals[parts[0]] + parts[1:]

        if len(parts) == 0:
            return

        action = parts[0]

        # ============================================================
        # CLEAR / CANCEL
        # ============================================================

        if action in ["CLEAR", "CLEAR_COMMAND", "CANCEL"]:

            self.command_active = False
            self.command_target = None
            self.command_text = None
            self.command_yaw = None
            self.yaw_command_active = False

            self.publish_command_status(
                "Command cancelled"
            )

            return

        # ============================================================
        # POSITION REQUIRED
        # ============================================================

        if self.current_position is None:

            self.publish_command_status(
                "REJECTED: position unavailable"
            )

            return

        # ============================================================
        # ONLY ACCEPT COMMANDS AFTER TAKEOFF
        # ============================================================

        if self.state in [
            "HEARTBEAT",
            "ARM",
            "OFFBOARD",
            "TAKEOFF",
            "LAND",
            "LANDING",
            "DONE",
        ]:

            self.publish_command_status(
                f"REJECTED: controller state is {self.state}"
            )

            return

        # ============================================================
        # HOVER
        # ============================================================

        if action == "HOVER":

            self.command_target = (
                float(self.current_position.x),
                float(self.current_position.y),
                float(self.current_position.z),
            )

            self.command_active = True
            self.command_text = command
            self.command_yaw = None
            self.yaw_command_active = False

            self.publish_command_status(
                "Hover command accepted"
            )

            return

        # ============================================================
        # RETURN HOME
        # ============================================================

        if action == "RETURN_HOME":

            self.command_active = False
            self.command_target = None
            self.command_yaw = None
            self.yaw_command_active = False

            self.state = "RETURN_HOME"

            self.publish_command_status(
                "Return home command accepted"
            )

            return

        # ============================================================
        # GOTO X Y Z
        # ============================================================

        if action == "GOTO":

            if len(parts) != 4:

                self.publish_command_status(
                    "ERROR: GO TO requires X Y Z"
                )

                return

            try:

                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])

            except ValueError:

                self.publish_command_status(
                    "ERROR: invalid coordinates"
                )

                return

            self.command_target = (
                x,
                y,
                z,
            )

            self.command_active = True
            self.command_text = command

            self.command_yaw = None
            self.yaw_command_active = False

            self.avoidance_active = False
            self.avoidance_target = None

            self.publish_command_status(
                f"Accepted GOTO: "
                f"({x:.2f}, {y:.2f}, {z:.2f})"
            )

            return

        # ============================================================
        # GET CURRENT POSITION
        # ============================================================

        x = float(self.current_position.x)
        y = float(self.current_position.y)
        z = float(self.current_position.z)

        # ============================================================
        # CARDINAL / RELATIVE MOVEMENT
        # ============================================================

        relative_commands = {
            "FORWARD": "FORWARD",
            "FRONT": "FORWARD",

            "BACK": "BACKWARD",
            "BACKWARD": "BACKWARD",

            "LEFT": "LEFT",
            "RIGHT": "RIGHT",

            "UP": "UP",
            "DOWN": "DOWN",

            "NORTH": "NORTH",
            "SOUTH": "SOUTH",
            "EAST": "EAST",
            "WEST": "WEST",
        }

        # ============================================================
        # DIAGONAL MOVEMENT
        #
        # NORTH EAST 20
        # NORTH WEST 20
        # SOUTH EAST 20
        # SOUTH WEST 20
        # ============================================================

        diagonal_commands = {

            ("NORTH", "EAST"): "NORTH_EAST",
            ("NORTH", "WEST"): "NORTH_WEST",

            ("SOUTH", "EAST"): "SOUTH_EAST",
            ("SOUTH", "WEST"): "SOUTH_WEST",
        }

        direction = None
        distance = None

        # ------------------------------------------------------------
        # Diagonal command
        # ------------------------------------------------------------

        if len(parts) == 3:

            direction_pair = (
                parts[0],
                parts[1],
            )

            if direction_pair in diagonal_commands:

                direction = diagonal_commands[
                    direction_pair
                ]

                try:

                    distance = float(parts[2])

                except ValueError:

                    self.publish_command_status(
                        "ERROR: invalid distance"
                    )

                    return

        # ------------------------------------------------------------
        # Normal direction command
        # ------------------------------------------------------------

        elif len(parts) == 2:

            if parts[0] in relative_commands:

                direction = relative_commands[
                    parts[0]
                ]

                try:

                    distance = float(parts[1])

                except ValueError:

                    self.publish_command_status(
                        "ERROR: invalid distance"
                    )

                    return

        # ============================================================
        # EXECUTE MOVEMENT
        # ============================================================

        if direction is not None:

            if distance is None or distance <= 0.0:

                self.publish_command_status(
                    "ERROR: distance must be positive"
                )

                return

            # --------------------------------------------------------
            # FORWARD / BACKWARD
            # --------------------------------------------------------

            if direction == "FORWARD":

                target = (
                    x + distance,
                    y,
                    z,
                )

            elif direction == "BACKWARD":

                target = (
                    x - distance,
                    y,
                    z,
                )

            # --------------------------------------------------------
            # LEFT / RIGHT
            # --------------------------------------------------------

            elif direction == "LEFT":

                target = (
                    x,
                    y - distance,
                    z,
                )

            elif direction == "RIGHT":

                target = (
                    x,
                    y + distance,
                    z,
                )

            # --------------------------------------------------------
            # UP / DOWN
            # PX4 NED
            # --------------------------------------------------------

            elif direction == "UP":

                target = (
                    x,
                    y,
                    z - distance,
                )

            elif direction == "DOWN":

                target = (
                    x,
                    y,
                    z + distance,
                )

            # --------------------------------------------------------
            # CARDINAL DIRECTIONS
            # --------------------------------------------------------

            elif direction == "NORTH":

                target = (
                    x + distance,
                    y,
                    z,
                )

            elif direction == "SOUTH":

                target = (
                    x - distance,
                    y,
                    z,
                )

            elif direction == "EAST":

                target = (
                    x,
                    y + distance,
                    z,
                )

            elif direction == "WEST":

                target = (
                    x,
                    y - distance,
                    z,
                )

            # --------------------------------------------------------
            # DIAGONALS
            #
            # Divide by sqrt(2) so the TOTAL movement distance
            # remains equal to the distance requested.
            # --------------------------------------------------------

            elif direction == "NORTH_EAST":

                diagonal = distance / math.sqrt(2.0)

                target = (
                    x + diagonal,
                    y + diagonal,
                    z,
                )

            elif direction == "NORTH_WEST":

                diagonal = distance / math.sqrt(2.0)

                target = (
                    x + diagonal,
                    y - diagonal,
                    z,
                )

            elif direction == "SOUTH_EAST":

                diagonal = distance / math.sqrt(2.0)

                target = (
                    x - diagonal,
                    y + diagonal,
                    z,
                )

            elif direction == "SOUTH_WEST":

                diagonal = distance / math.sqrt(2.0)

                target = (
                    x - diagonal,
                    y - diagonal,
                    z,
                )

            else:

                self.publish_command_status(
                    "ERROR: unknown direction"
                )

                return

            self.command_target = target
            self.command_active = True
            self.command_text = command

            self.command_yaw = None
            self.yaw_command_active = False

            self.avoidance_active = False
            self.avoidance_target = None

            self.publish_command_status(
                f"Accepted {command} -> "
                f"target "
                f"({target[0]:.2f}, "
                f"{target[1]:.2f}, "
                f"{target[2]:.2f})"
            )

            return

        # ============================================================
        # TURN LEFT / TURN RIGHT
        # ============================================================

        if action in ["TURN_LEFT", "TURN_RIGHT"]:

            if len(parts) != 2:

                self.publish_command_status(
                    "ERROR: TURN requires angle"
                )

                return

            try:

                degrees = float(parts[1])

            except ValueError:

                self.publish_command_status(
                    "ERROR: invalid angle"
                )

                return

            if degrees <= 0.0:

                self.publish_command_status(
                    "ERROR: angle must be positive"
                )

                return

            current_yaw = self.get_current_yaw()

            if current_yaw is None:

                self.publish_command_status(
                    "ERROR: attitude/yaw unavailable"
                )

                return

            delta = math.radians(degrees)

            if action == "TURN_LEFT":

                target_yaw = current_yaw - delta

            else:

                target_yaw = current_yaw + delta

            target_yaw = self.normalize_angle(
                target_yaw
            )

            self.command_target = (
                x,
                y,
                z,
            )

            self.command_active = True
            self.command_text = command

            self.command_yaw = target_yaw
            self.yaw_command_active = True

            self.avoidance_active = False
            self.avoidance_target = None

            self.publish_command_status(
                f"Accepted {command}"
            )

            return

        # ============================================================
        # LAND
        # ============================================================

        if action == "LAND":

            self.command_active = False
            self.command_target = None
            self.command_yaw = None
            self.yaw_command_active = False

            self.state = "LAND"

            self.publish_command_status(
                "Landing command accepted"
            )

            return

        # ============================================================
        # UNKNOWN COMMAND
        # ============================================================

        self.publish_command_status(
            "ERROR: unknown command"
        )
    # ================================================================
    # YAW HELPERS
    # ================================================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def get_current_yaw(self):

        # VehicleLocalPosition heading is already available
        # from PX4 and is preferable for this controller.

        if self.current_position is None:
            return None

        try:
            return float(self.current_position.heading)

        except Exception:
            return None

    def yaw_reached(self):

        if not self.yaw_command_active:
            return True

        if self.current_position is None:
            return False

        current_yaw = self.get_current_yaw()

        if current_yaw is None:
            return False

        error = self.normalize_angle(
            self.command_yaw - current_yaw
        )

        return abs(error) < math.radians(5.0)

    # ================================================================
    # PX4 HELPERS
    # ================================================================

    def px4_timestamp(self):

        return (
            self.get_clock().now().nanoseconds
            // 1000
        )

    def publish_offboard(self):

        # Monotonic diagnostics for the actual Offboard publish path.
        _now = time.monotonic()
        _last = getattr(self, "_offboard_publish_last_monotonic", None)
        _seq = getattr(self, "_offboard_publish_seq", 0) + 1
        self._offboard_publish_seq = _seq

        if _last is not None:
            _gap = _now - _last

            if _gap > 0.3:
                self.get_logger().warn(
                    f"OFFBOARD PUBLISH GAP: {_gap:.3f}s seq={_seq}"
                )

        self._offboard_publish_last_monotonic = _now

        _px4_timestamp = self.px4_timestamp()

        # Detect ROS/PX4 timestamp discontinuities without changing them.
        _last_px4_timestamp = getattr(
            self, "_last_offboard_px4_timestamp", None
        )

        if (
            _last_px4_timestamp is not None
            and _px4_timestamp <= _last_px4_timestamp
        ):
            self.get_logger().warn(
                "OFFBOARD PX4 TIMESTAMP NON-MONOTONIC: "
                f"current={_px4_timestamp} "
                f"previous={_last_px4_timestamp}"
            )

        self._last_offboard_px4_timestamp = _px4_timestamp

        msg = OffboardControlMode()

        msg.timestamp = _px4_timestamp

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        _publish_start = time.monotonic()
        self.offboard_pub.publish(msg)
        _publish_duration = time.monotonic() - _publish_start

        if _publish_duration > 0.2:
            self.get_logger().warn(
                f"OFFBOARD PUBLISH BLOCKED: {_publish_duration:.3f}s "
                f"seq={_seq}"
            )

    def publish_waypoint(
        self,
        x,
        y,
        z,
        yaw=None,
    ):

        msg = TrajectorySetpoint()

        msg.timestamp = self.px4_timestamp()

        msg.position = [
            float(x),
            float(y),
            float(z),
        ]

        if yaw is None:

            # Preserve current yaw.

            msg.yaw = float("nan")

        else:

            msg.yaw = float(yaw)

        self.trajectory_pub.publish(msg)

    def vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
    ):

        msg = VehicleCommand()

        msg.timestamp = self.px4_timestamp()

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.command = command

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.confirmation = 0
        msg.from_external = True

        self.command_pub.publish(msg)

        self.get_logger().info(
            f"VehicleCommand sent: {command}"
        )

    # ================================================================
    # POSITION HELPERS
    # ================================================================

    def distance_to(self, target):

        if self.current_position is None:
            return float("inf")

        dx = self.current_position.x - target[0]
        dy = self.current_position.y - target[1]
        dz = self.current_position.z - target[2]

        return math.sqrt(
            dx * dx +
            dy * dy +
            dz * dz
        )

    def waypoint_reached(self, waypoint):

        return (
            self.distance_to(waypoint)
            <= self.position_tolerance
        )

    # ================================================================
    # SENSOR VALIDITY
    # ================================================================

    def sensor_is_fresh(self):

        if not self.clearance_received:
            return False

        if self.last_obstacle_time is None:
            return False

        age = (
            self.get_clock().now()
            - self.last_obstacle_time
        ).nanoseconds / 1e9

        return age <= self.sensor_timeout

    # ================================================================
    # OBSTACLE DETECTION
    # ================================================================

    def obstacle_detected(self):

        if not self.sensor_is_fresh():
            return False

        forward = self.clearance[0]

        if forward <= 0.0:
            return False

        return forward < self.obstacle_threshold

    # ================================================================
    # WAYPOINT PROGRESS SCORE
    # ================================================================

    def direction_progress_score(
        self,
        direction,
    ):

        if self.current_position is None:
            return 0.0

        if self.original_target is None:
            return 0.0

        dx = (
            self.original_target[0]
            - self.current_position.x
        )

        dy = (
            self.original_target[1]
            - self.current_position.y
        )

        dz = (
            self.original_target[2]
            - self.current_position.z
        )

        distance = math.sqrt(
            dx * dx +
            dy * dy +
            dz * dz
        )

        if distance < 0.001:
            return 0.0

        dx /= distance
        dy /= distance
        dz /= distance

        if direction == "FORWARD":
            return max(0.0, dx)

        if direction == "BACKWARD":
            return max(0.0, -dx)

        if direction == "LEFT":
            return max(0.0, -dy)

        if direction == "RIGHT":
            return max(0.0, dy)

        if direction == "UP":
            return max(0.0, -dz)

        if direction == "DOWN":
            return max(0.0, dz)

        return 0.0

    # ================================================================
    # CHOOSE ESCAPE DIRECTION
    # ================================================================

    def choose_avoidance_direction(self):

        if self.current_position is None:
            return None

        (
            forward,
            backward,
            left,
            right,
            up,
            down,
        ) = self.clearance

        # ============================================================
        # USE BACKEND DECISION FIRST
        #
        # The backend decision is generated from:
        #     LiDAR + Radar -> Fusion -> Avoidance Controller
        # ============================================================

        decision = self.avoidance_decision

        if decision in [
            "TURN_LEFT",
            "MOVE_LEFT",
            "AVOID_LEFT",
            "LEFT",
        ]:
            if left >= self.minimum_escape_clearance:
                return "LEFT"

        if decision in [
            "TURN_RIGHT",
            "MOVE_RIGHT",
            "AVOID_RIGHT",
            "RIGHT",
        ]:
            if right >= self.minimum_escape_clearance:
                return "RIGHT"

        if decision in [
            "MOVE_BACKWARD",
            "AVOID_BACKWARD",
            "BACKWARD",
        ]:
            if backward >= self.minimum_escape_clearance:
                return "BACKWARD"

        if decision in [
            "MOVE_FORWARD",
            "AVOID_FORWARD",
            "FORWARD",
        ]:
            if forward >= self.minimum_escape_clearance:
                return "FORWARD"

        if decision in [
            "MOVE_UP",
            "AVOID_UP",
            "UP",
        ]:
            if up >= self.minimum_escape_clearance:
                return "UP"

        if decision in [
            "MOVE_DOWN",
            "AVOID_DOWN",
            "DOWN",
        ]:
            if down >= self.minimum_escape_clearance:
                return "DOWN"

        # ============================================================
        # FALLBACK
        #
        # If the backend decision is unavailable or its selected
        # direction is blocked, choose the safest available direction.
        # ============================================================

        candidates = []

        def add_candidate(direction, available):

            if available <= 0.0:
                return

            if available < self.minimum_escape_clearance:
                return

            usable = available - self.safety_margin

            if usable <= 0.0:
                return

            progress = self.direction_progress_score(direction)

            score = (
                usable * 2.0
                + progress * 1.5
            )

            if direction == self.avoidance_direction:
                score += 0.5

            candidates.append(
                (
                    direction,
                    available,
                    usable,
                    score,
                )
            )

        add_candidate("FORWARD", forward)
        add_candidate("BACKWARD", backward)
        add_candidate("LEFT", left)
        add_candidate("RIGHT", right)
        add_candidate("UP", up)
        add_candidate("DOWN", down)

        if not candidates:
            return None

        candidates.sort(
            key=lambda item: item[3],
            reverse=True,
        )

        return candidates[0][0]

    # ================================================================
    # DYNAMIC MOVEMENT DISTANCE
    # ================================================================

    def calculate_movement(
        self,
        available,
    ):

        if available <= 0.0:
            return 0.0

        usable = (
            available
            - self.safety_margin
        )

        if usable <= 0.0:
            return 0.0

        movement = usable * 0.70

        movement = max(
            self.minimum_avoidance_step,
            movement,
        )

        movement = min(
            self.max_avoidance_step,
            movement,
        )

        return movement

    # ================================================================
    # CREATE AVOIDANCE TARGET
    # ================================================================

    def create_avoidance_target(
        self,
        direction,
    ):

        if self.current_position is None:
            return None

        (
            forward,
            backward,
            left,
            right,
            up,
            down,
        ) = self.clearance

        x = float(
            self.current_position.x
        )

        y = float(
            self.current_position.y
        )

        z = float(
            self.current_position.z
        )

        available = None

        # ------------------------------------------------------------
        # FORWARD
        # ------------------------------------------------------------

        if direction == "FORWARD":

            available = forward

            movement = self.calculate_movement(
                available
            )

            if movement <= 0.0:
                return None

            target = (
                x + movement,
                y,
                z,
            )

        # ------------------------------------------------------------
        # BACKWARD
        # ------------------------------------------------------------

        elif direction == "BACKWARD":

            available = backward

            movement = self.calculate_movement(
                available
            )

            if movement <= 0.0:
                return None

            target = (
                x - movement,
                y,
                z,
            )

        # ------------------------------------------------------------
        # LEFT
        # ------------------------------------------------------------

        elif direction == "LEFT":

            available = left

            movement = self.calculate_movement(
                available
            )

            if movement <= 0.0:
                return None

            target = (
                x,
                y - movement,
                z,
            )

        # ------------------------------------------------------------
        # RIGHT
        # ------------------------------------------------------------

        elif direction == "RIGHT":

            available = right

            movement = self.calculate_movement(
                available
            )

            if movement <= 0.0:
                return None

            target = (
                x,
                y + movement,
                z,
            )

        # ------------------------------------------------------------
        # UP
        # ------------------------------------------------------------

        elif direction == "UP":

            available = up

            movement = self.calculate_movement(
                available
            )

            if movement <= 0.0:
                return None

            # PX4 NED:
            # smaller Z = higher altitude.

            target = (
                x,
                y,
                z - movement,
            )

        # ------------------------------------------------------------
        # DOWN
        # ------------------------------------------------------------

        elif direction == "DOWN":

            available = down

            movement = self.calculate_movement(
                available
            )

            if movement <= 0.0:
                return None

            # PX4 NED:
            # larger Z = lower altitude.

            target = (
                x,
                y,
                z + movement,
            )

        else:

            return None

        self.get_logger().warn(
            f"Dynamic movement: "
            f"{movement:.2f} m "
            f"from clearance "
            f"{available:.2f} m"
        )

        return target

    # ================================================================
    # START AVOIDANCE
    # ================================================================

    def start_avoidance(
        self,
        original_target,
    ):

        if self.avoidance_active:
            return

        self.original_target = original_target

        direction = (
            self.choose_avoidance_direction()
        )

        if direction is None:

            self.get_logger().error(
                "Obstacle detected but "
                "no safe escape direction."
            )

            return

        target = (
            self.create_avoidance_target(
                direction
            )
        )

        if target is None:

            self.get_logger().error(
                "Could not create avoidance target."
            )

            return

        self.avoidance_direction = direction

        self.avoidance_target = target

        self.avoidance_active = True

        self.obstacle_clear_counter = 0

        self.get_logger().warn(
            "======================================"
        )

        self.get_logger().warn(
            f"DYNAMIC AVOIDANCE STARTED: "
            f"{direction}"
        )

        self.get_logger().warn(
            f"Original target: "
            f"({original_target[0]:.2f}, "
            f"{original_target[1]:.2f}, "
            f"{original_target[2]:.2f})"
        )

        self.get_logger().warn(
            f"Temporary target: "
            f"({target[0]:.2f}, "
            f"{target[1]:.2f}, "
            f"{target[2]:.2f})"
        )

        self.get_logger().warn(
            "======================================"
        )

    # ================================================================
    # OBSTACLE CLEARED
    # ================================================================

    def obstacle_cleared(self):

        if not self.sensor_is_fresh():
            return False

        forward = self.clearance[0]

        if forward <= 0.0:
            return False

        required_distance = (
            self.obstacle_threshold
            + self.clear_distance
        )

        return (
            forward >= required_distance
        )

    # ================================================================
    # EXECUTE DYNAMIC AVOIDANCE
    # ================================================================

    def execute_avoidance(self):

        if not self.avoidance_active:
            return False

        if self.avoidance_target is None:

            self.avoidance_active = False

            return False

        self.publish_waypoint(
            *self.avoidance_target
        )

        # ------------------------------------------------------------
        # Temporary target not reached
        # ------------------------------------------------------------

        if not self.waypoint_reached(
            self.avoidance_target
        ):

            # Re-plan if another obstacle is
            # detected while avoiding.

            if self.obstacle_detected():

                self.get_logger().warn(
                    "Obstacle detected while "
                    "moving to avoidance target - "
                    "re-planning."
                )

                direction = (
                    self.choose_avoidance_direction()
                )

                if direction is not None:

                    target = (
                        self.create_avoidance_target(
                            direction
                        )
                    )

                    if target is not None:

                        self.avoidance_direction = (
                            direction
                        )

                        self.avoidance_target = (
                            target
                        )

            return True

        # ------------------------------------------------------------
        # Temporary target reached
        # ------------------------------------------------------------

        if self.obstacle_cleared():

            self.obstacle_clear_counter += 1

            if self.obstacle_clear_counter == 1:

                self.get_logger().info(
                    "Avoidance point reached. "
                    "Forward path appears clear."
                )

            if (
                self.obstacle_clear_counter
                >= self.required_clear_cycles
            ):

                self.get_logger().info(
                    "Obstacle cleared. "
                    "Resuming original target."
                )

                self.avoidance_active = False

                self.avoidance_target = None

                self.avoidance_direction = None

                self.obstacle_clear_counter = 0

                return False

        else:

            self.obstacle_clear_counter = 0

            self.get_logger().warn(
                "Obstacle still blocking path. "
                "Re-planning from current position."
            )

            direction = (
                self.choose_avoidance_direction()
            )

            if direction is None:

                self.get_logger().error(
                    "NO SAFE ESCAPE DIRECTION. "
                    "Holding current position."
                )

                if self.current_position is not None:

                    self.avoidance_target = (
                        float(
                            self.current_position.x
                        ),
                        float(
                            self.current_position.y
                        ),
                        float(
                            self.current_position.z
                        ),
                    )

                return True

            target = (
                self.create_avoidance_target(
                    direction
                )
            )

            if target is not None:

                self.avoidance_direction = (
                    direction
                )

                self.avoidance_target = (
                    target
                )

                self.get_logger().warn(
                    f"New avoidance target: "
                    f"({target[0]:.2f}, "
                    f"{target[1]:.2f}, "
                    f"{target[2]:.2f})"
                )

        return True

    # ================================================================
    # COMMAND EXECUTION
    # ================================================================

    def execute_external_command(self):

        if not self.command_active:
            return False

        if self.command_target is None:

            self.command_active = False

            return False

        # ------------------------------------------------------------
        # If an obstacle is detected, dynamic avoidance
        # gets priority over the external command.
        # ------------------------------------------------------------

        if self.avoidance_active:

            if self.execute_avoidance():

                return True

        # ------------------------------------------------------------
        # Start avoidance for command target
        # ------------------------------------------------------------

        if self.obstacle_detected():

            self.start_avoidance(
                self.command_target
            )

            if self.avoidance_active:

                return True

        # ------------------------------------------------------------
        # TURN command
        # ------------------------------------------------------------

        if self.yaw_command_active:

            self.publish_waypoint(
                self.command_target[0],
                self.command_target[1],
                self.command_target[2],
                self.command_yaw,
            )

            if self.yaw_reached():

                self.get_logger().info(
                    f"TURN COMPLETE: "
                    f"{self.command_text}"
                )

                self.publish_command_status(
                    f"COMPLETED: "
                    f"{self.command_text}"
                )

                self.command_active = False
                self.command_target = None
                self.command_text = None
                self.command_yaw = None
                self.yaw_command_active = False

                return False

            return True

        # ------------------------------------------------------------
        # Normal movement/GOTO command
        # ------------------------------------------------------------

        self.publish_waypoint(
            *self.command_target
        )

        if self.waypoint_reached(
            self.command_target
        ):

            self.get_logger().info(
                f"COMMAND COMPLETE: "
                f"{self.command_text}"
            )

            self.publish_command_status(
                f"COMPLETED: "
                f"{self.command_text}"
            )

            self.command_active = False
            self.command_target = None
            self.command_text = None

            return False

        return True

    # ================================================================
    # STATE MACHINE
    # ================================================================

    def timer_callback(self):
        _timer_start = time.monotonic()
        _timer_gap = _timer_start - getattr(self, "_last_timer_entry", _timer_start)
        self._last_timer_entry = _timer_start

        if _timer_gap > 0.3:
            self.get_logger().warn(
                f"TIMER ENTRY GAP: {_timer_gap:.3f}s"
            )

        # Offboard heartbeat MUST continue.

        self.publish_offboard()

        # ============================================================
        # HEARTBEAT
        # ============================================================

        if self.state == "HEARTBEAT":

            self.publish_waypoint(
                *self.waypoints[0]
            )

            self.heartbeat_count += 1

            if self.heartbeat_count >= 30:

                self.get_logger().info(
                    "Heartbeat complete - "
                    "sending ARM command"
                )

                self.state = "ARM"

            return

        # ============================================================
        # ARM
        # ============================================================

        if self.state == "ARM":

            self.publish_waypoint(
                *self.waypoints[0]
            )

            if not self.arm_command_sent:

                self.get_logger().info(
                    "Sending ARM command"
                )

                self.vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0,
                )

                self.arm_command_sent = True

            self.state = "OFFBOARD"

            return

        # ============================================================
        # OFFBOARD
        # ============================================================

        if self.state == "OFFBOARD":

            self.publish_waypoint(
                *self.waypoints[0]
            )

            if not self.offboard_command_sent:

                self.get_logger().info(
                    "Switching to OFFBOARD mode"
                )

                self.vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,
                    param2=6.0,
                )

                self.offboard_command_sent = True

            self.state = "TAKEOFF"

            return

        # ============================================================
        # TAKEOFF
        # ============================================================

        if self.state == "TAKEOFF":

            if (
                self.home_x is None
                or self.current_position is None
            ):

                return

            takeoff_target = (
                self.home_x,
                self.home_y,
                self.waypoints[0][2],
            )

            self.publish_waypoint(
                *takeoff_target
            )

            if self.waypoint_reached(
                takeoff_target
            ):

                self.waypoint_hold_cycles += 1

                if self.waypoint_hold_cycles == 1:

                    self.get_logger().info(
                        "Takeoff altitude reached - "
                        "holding"
                    )

                if (
                    self.waypoint_hold_cycles
                    >= self.waypoint_hold_cycles_required
                ):

                    self.current_waypoint = 0

                    self.waypoint_hold_cycles = 0

                    self.get_logger().info(
                        "Takeoff complete - "
                        "starting waypoint mission"
                    )

                    self.state = "WAYPOINT"

            else:

                self.waypoint_hold_cycles = 0

            return

        # ============================================================
        # WAYPOINT MISSION
        # ============================================================

        if self.state == "WAYPOINT":

            # --------------------------------------------------------
            # EXTERNAL COMMAND HAS PRIORITY
            # --------------------------------------------------------

            if self.command_active:

                self.execute_external_command()

                return

            # --------------------------------------------------------
            # Mission completed
            # --------------------------------------------------------

            if (
                self.current_waypoint
                >= len(self.waypoints)
            ):

                self.get_logger().info(
                    "All waypoints completed - "
                    "returning home"
                )

                self.return_home_hold_cycles = 0

                self.state = "RETURN_HOME"

                return

            target = self.waypoints[
                self.current_waypoint
            ]

            # --------------------------------------------------------
            # Continue active obstacle avoidance
            # --------------------------------------------------------

            if self.avoidance_active:

                if self.execute_avoidance():

                    return

            # --------------------------------------------------------
            # Detect obstacle before moving
            # --------------------------------------------------------

            if self.obstacle_detected():

                self.start_avoidance(
                    target
                )

                if self.avoidance_active:

                    return

            # --------------------------------------------------------
            # Normal waypoint navigation
            # --------------------------------------------------------

            self.publish_waypoint(
                *target
            )

            if self.waypoint_reached(
                target
            ):

                self.waypoint_hold_cycles += 1

                if self.waypoint_hold_cycles == 1:

                    self.get_logger().info(
                        f"Waypoint "
                        f"{self.current_waypoint} "
                        f"reached: "
                        f"({target[0]:.1f}, "
                        f"{target[1]:.1f}, "
                        f"{target[2]:.1f})"
                    )

                if (
                    self.waypoint_hold_cycles
                    >= self.waypoint_hold_cycles_required
                ):

                    self.get_logger().info(
                        f"Held waypoint "
                        f"{self.current_waypoint}"
                    )

                    self.current_waypoint += 1

                    self.waypoint_hold_cycles = 0

                    if (
                        self.current_waypoint
                        >= len(self.waypoints)
                    ):

                        self.get_logger().info(
                            "FINAL WAYPOINT COMPLETE - "
                            "returning to base"
                        )

                        self.return_home_hold_cycles = 0

                        self.state = "RETURN_HOME"

                    else:

                        next_target = (
                            self.waypoints[
                                self.current_waypoint
                            ]
                        )

                        self.get_logger().info(
                            f"Moving to waypoint "
                            f"{self.current_waypoint}: "
                            f"({next_target[0]:.1f}, "
                            f"{next_target[1]:.1f}, "
                            f"{next_target[2]:.1f})"
                        )

            else:

                self.waypoint_hold_cycles = 0

            return

        # ============================================================
        # RETURN HOME
        # ============================================================

        if self.state == "RETURN_HOME":

            if (
                self.home_x is None
                or self.current_position is None
            ):

                return

            home_target = (
                self.home_x,
                self.home_y,
                self.waypoints[0][2],
            )

            # Dynamic avoidance during return.

            if self.avoidance_active:

                if self.execute_avoidance():

                    return

            if self.obstacle_detected():

                self.start_avoidance(
                    home_target
                )

                if self.avoidance_active:

                    return

            self.publish_waypoint(
                *home_target
            )

            if self.waypoint_reached(
                home_target
            ):

                self.return_home_hold_cycles += 1

                if self.return_home_hold_cycles == 1:

                    self.get_logger().info(
                        f"Base reached - holding at "
                        f"({self.home_x:.1f}, "
                        f"{self.home_y:.1f}, "
                        f"{self.waypoints[0][2]:.1f})"
                    )

                if (
                    self.return_home_hold_cycles
                    >= self.waypoint_hold_cycles_required
                ):

                    self.return_home_hold_cycles = 0

                    self.get_logger().info(
                        "Return-to-base complete - "
                        "starting landing"
                    )

                    self.state = "LAND"

            else:

                self.return_home_hold_cycles = 0

            return

        # ============================================================
        # LAND
        # ============================================================

        if self.state == "LAND":

            if self.home_x is not None:

                self.publish_waypoint(
                    self.home_x,
                    self.home_y,
                    self.waypoints[0][2],
                )

            if not self.land_command_sent:

                self.get_logger().info(
                    "Sending LAND command"
                )

                self.vehicle_command(
                    VehicleCommand.VEHICLE_CMD_NAV_LAND
                )

                self.land_command_sent = True

                self.state = "LANDING"

            return

        # ============================================================
        # LANDING
        # ============================================================

        if self.state == "LANDING":

            if self.vehicle_status is not None:

                # PX4 arming_state 1 = disarmed.

                if (
                    self.vehicle_status.arming_state
                    == 1
                ):

                    self.get_logger().info(
                        "Drone disarmed - "
                        "landing complete"
                    )

                    self.state = "DONE"

            return

        # ============================================================
        # DONE
        # ============================================================

        if self.state == "DONE":

            pass


def main(args=None):

    rclpy.init(args=args)

    node = OffboardController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "Controller stopped by user"
        )

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
