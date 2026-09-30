import math


class TelemetryManager:

    @staticmethod
    def quaternion_to_euler(q):

        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]

        # Roll
        sin_roll = 2.0 * (
            w * x + y * z
        )

        cos_roll = 1.0 - 2.0 * (
            x * x + y * y
        )

        roll = math.atan2(
            sin_roll,
            cos_roll
        )

        # Pitch
        sin_pitch = 2.0 * (
            w * y - z * x
        )

        sin_pitch = max(
            -1.0,
            min(1.0, sin_pitch)
        )

        pitch = math.asin(
            sin_pitch
        )

        # Yaw
        sin_yaw = 2.0 * (
            w * z + x * y
        )

        cos_yaw = 1.0 - 2.0 * (
            y * y + z * z
        )

        yaw = math.atan2(
            sin_yaw,
            cos_yaw
        )

        return (
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
        )
