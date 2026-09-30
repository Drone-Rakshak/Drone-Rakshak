import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, NavSatFix


class SensorMonitor(Node):

    def __init__(self):
        super().__init__("sensor_monitor")

        self.latest_imu = None
        self.latest_gps = None

        self.create_subscription(
            Imu,
            "/world/drone_rakshak/model/x500_0/link/base_link/sensor/imu_sensor/imu",
            self.imu_callback,
            10,
        )

        self.create_subscription(
            NavSatFix,
            "/world/drone_rakshak/model/x500_0/link/base_link/sensor/navsat_sensor/navsat",
            self.gps_callback,
            10,
        )

        self.create_timer(1.0, self.print_status)

    def imu_callback(self, msg):
        self.latest_imu = msg

    def gps_callback(self, msg):
        self.latest_gps = msg

    def print_status(self):

        if self.latest_imu is None or self.latest_gps is None:
            return

        self.get_logger().info(
            f"""
================ Drone Status ================

GPS:
  Latitude  : {self.latest_gps.latitude:.6f}
  Longitude : {self.latest_gps.longitude:.6f}
  Altitude  : {self.latest_gps.altitude:.2f} m

IMU:
  Accel Z   : {self.latest_imu.linear_acceleration.z:.2f} m/s²
  Gyro Z    : {self.latest_imu.angular_velocity.z:.4f} rad/s

==============================================
"""
        )


def main(args=None):
    rclpy.init(args=args)

    node = SensorMonitor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
