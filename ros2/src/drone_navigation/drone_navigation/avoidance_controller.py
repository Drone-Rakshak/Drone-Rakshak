#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, String


class AvoidanceController(Node):

    def __init__(self):

        super().__init__("avoidance_controller")

        # ============================================================
        # STARTUP
        # ============================================================

        

        # ============================================================
        # CONFIGURATION
        # ============================================================

        # Distance below which an obstacle requires avoidance.
        self.danger_distance = 2.0

        # Distance below which an obstacle is critical.
        self.critical_distance = 1.0

        # Maximum sensor range.
        self.max_range = 30.0

        # ============================================================
        # SENSOR STATE
        # ============================================================

        # Fused clearance:
        #
        # [Forward, Backward, Left, Right, Up, Down]
        #
        self.clearance = None

        # Latest radar aerial threat.
        self.radar_threat = "NONE"

        # Previous decision.
        #
        # Used to prevent continuous terminal spam.
        self.last_decision = None

        # ============================================================
        # SUBSCRIBE: LIDAR + RADAR FUSED CLEARANCE
        # ============================================================

        self.clearance_sub = self.create_subscription(
            Float32MultiArray,
            "/drone/fused_clearance",
            self.clearance_callback,
            10
        )

        # ============================================================
        # SUBSCRIBE: RADAR AERIAL THREAT
        # ============================================================

        self.radar_sub = self.create_subscription(
            String,
            "/drone/radar_threat",
            self.radar_callback,
            10
        )

        # ============================================================
        # PUBLISH AVOIDANCE DECISION
        # ============================================================

        self.decision_pub = self.create_publisher(
            String,
            "/drone/avoidance_decision",
            10
        )

        # ============================================================
        # DECISION LOOP
        # ============================================================

        self.timer = self.create_timer(
            0.1,
            self.decision_loop
        )

    # ================================================================
    # FUSED CLEARANCE CALLBACK
    # ================================================================

    def clearance_callback(self, msg):

        if len(msg.data) < 6:

            return

        self.clearance = [
            float(msg.data[0]),  # Forward
            float(msg.data[1]),  # Backward
            float(msg.data[2]),  # Left
            float(msg.data[3]),  # Right
            float(msg.data[4]),  # Up
            float(msg.data[5]),  # Down
        ]

    # ================================================================
    # RADAR THREAT CALLBACK
    # ================================================================

    def radar_callback(self, msg):

        self.radar_threat = msg.data.strip()

    # ================================================================
    # DECISION LOOP
    # ================================================================

    def decision_loop(self):

        # ------------------------------------------------------------
        # Wait for sensor data
        # ------------------------------------------------------------

        if self.clearance is None:

            self.publish_decision(
                "WAITING_FOR_SENSOR_DATA"
            )

            return

        # ------------------------------------------------------------
        # Read fused clearance
        # ------------------------------------------------------------

        forward = self.clearance[0]
        backward = self.clearance[1]
        left = self.clearance[2]
        right = self.clearance[3]
        up = self.clearance[4]
        down = self.clearance[5]

        # ============================================================
        # 1. RADAR AERIAL THREAT
        # ============================================================
        #
        # Aerial threats are handled separately from static
        # obstacles.
        #
        # We do NOT treat an aerial target as a wall.
        #
        # PX4 response will be implemented later.
        # ============================================================

        threat = self.radar_threat.upper()

        if (
            "AERIAL TARGET" in threat
            or "AERIAL THREAT" in threat
            or "ENEMY" in threat
            or "TARGET" in threat
        ):

            self.publish_decision(
                "RADAR_AERIAL_THREAT_DETECTED"
            )

            return

        # ============================================================
        # 2. CRITICAL FRONT OBSTACLE
        # ============================================================

        if forward < self.critical_distance:

            if left > right:

                decision = (
                    "CRITICAL_FRONT_OBSTACLE_TURN_LEFT"
                )

            elif right > left:

                decision = (
                    "CRITICAL_FRONT_OBSTACLE_TURN_RIGHT"
                )

            else:

                decision = (
                    "CRITICAL_FRONT_OBSTACLE_HOLD"
                )

            self.publish_decision(decision)

            return

        # ============================================================
        # 3. FRONT OBSTACLE
        # ============================================================

        if forward < self.danger_distance:

            if left > right:

                decision = (
                    "FRONT_OBSTACLE_TURN_LEFT"
                )

            elif right > left:

                decision = (
                    "FRONT_OBSTACLE_TURN_RIGHT"
                )

            else:

                decision = (
                    "FRONT_OBSTACLE_HOLD"
                )

            self.publish_decision(decision)

            return

        # ============================================================
        # 4. LEFT OBSTACLE
        # ============================================================

        if left < self.danger_distance:

            self.publish_decision(
                "LEFT_OBSTACLE_MOVE_RIGHT"
            )

            return

        # ============================================================
        # 5. RIGHT OBSTACLE
        # ============================================================

        if right < self.danger_distance:

            self.publish_decision(
                "RIGHT_OBSTACLE_MOVE_LEFT"
            )

            return

        # ============================================================
        # 6. BACKWARD OBSTACLE
        # ============================================================

        if backward < self.danger_distance:

            # Forward is clear because the front checks above
            # have already passed.

            self.publish_decision(
                "BACKWARD_OBSTACLE_AVOID_FORWARD"
            )

            return

        # ============================================================
        # 7. UP OBSTACLE
        # ============================================================
        #
        # Important:
        #
        # The current LiDAR is 2D, therefore U/D normally remain
        # at the default value of 30 m.
        #
        # We only react if the fusion system actually provides
        # a value below the danger threshold.
        # ============================================================

        if (
            up < self.danger_distance
            and up < self.max_range
        ):

            self.publish_decision(
                "UP_BLOCKED"
            )

            return

        # ============================================================
        # 8. DOWN OBSTACLE
        # ============================================================

        if (
            down < self.danger_distance
            and down < self.max_range
        ):

            self.publish_decision(
                "DOWN_BLOCKED"
            )

            return

        # ============================================================
        # 9. EVERYTHING CLEAR
        # ============================================================

        self.publish_decision(
            "PATH_CLEAR"
        )

    # ================================================================
    # PUBLISH DECISION
    # ================================================================

    def publish_decision(self, decision):

        # ------------------------------------------------------------
        # Always publish the current decision.
        #
        # This keeps the ROS topic continuously updated.
        # ------------------------------------------------------------

        msg = String()
        msg.data = decision

        self.decision_pub.publish(msg)

        # ------------------------------------------------------------
        # Only print when the decision changes.
        #
        # This prevents terminal spam.
        # ------------------------------------------------------------

        if decision == self.last_decision:

            return

        # ============================================================
        # LOGGING
        # ============================================================

        if decision == "PATH_CLEAR":

            self.get_logger().info(
                "PATH CLEAR"
            )

        elif decision == "WAITING_FOR_SENSOR_DATA":

            self.get_logger().info(
                "WAITING FOR SENSOR DATA"
            )

        elif decision == "RADAR_AERIAL_THREAT_DETECTED":

            self.get_logger().warn(
                "RADAR AERIAL THREAT DETECTED"
            )

        elif "CRITICAL" in decision:

            self.get_logger().error(
                f"AVOIDANCE: {decision}"
            )

        else:

            self.get_logger().warn(
                f"AVOIDANCE: {decision}"
            )

        self.last_decision = decision


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = AvoidanceController()

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
