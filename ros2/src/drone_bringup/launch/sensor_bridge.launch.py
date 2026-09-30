from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    # ============================================================
    # ROS-GZ BRIDGE
    # ============================================================

    bridge = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",

            # ----------------------------------------------------
            # PX4 / X500 CORE SENSORS
            # ----------------------------------------------------

            "/world/drone_rakshak/model/x500_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",

            "/world/drone_rakshak/model/x500_0/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",

            "/world/drone_rakshak/model/x500_0/link/base_link/sensor/magnetometer_sensor/magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer",

            "/world/drone_rakshak/model/x500_0/link/base_link/sensor/air_pressure_sensor/air_pressure@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure",

            # ----------------------------------------------------
            # RGB CAMERA
            # mono_cam integrated directly into x500
            # ----------------------------------------------------

            "/world/drone_rakshak/model/x500_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",

            # ----------------------------------------------------
            # 2D LIDAR
            # lidar_2d_v2 integrated directly into x500
            # ----------------------------------------------------

            "/world/drone_rakshak/model/x500_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        ],
        output="log"
    )

    # ============================================================
    # CAMERA TF
    # ============================================================

    camera_tf = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",

            "0.12",
            "0.03",
            "0.242",

            "0",
            "0",
            "0",

            "base_link",
            "camera_link"
        ],
        output="log"
    )

    # ============================================================
    # OBSTACLE DETECTOR
    # ============================================================

    obstacle_detector = ExecuteProcess(
        cmd=[
            "python3",
            "/home/vansh/drone_ws/src/drone_navigation/drone_navigation/obstacle_detector.py"
        ],
        output="screen"
    )

    # ============================================================
    # RADAR RECEIVER
    # ============================================================

    radar_receiver = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "drone_perception",
            "radar_receiver"
        ],
        output="log"
    )

    # ============================================================
    # RADAR + LIDAR FUSION
    # ============================================================

    radar_fusion = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "drone_perception",
            "radar_fusion"
        ],
        output="log"
    )

    # ============================================================
    # AVOIDANCE CONTROLLER
    # ============================================================

    avoidance_controller = ExecuteProcess(
        cmd=[
            "python3",
            "/home/vansh/drone_ws/src/drone_navigation/drone_navigation/avoidance_controller.py"
        ],
        output="log"
    )

    # ============================================================
    # FRAME PROCESSOR
    # ============================================================

    frame_processor = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "drone_perception",
            "frame_processor"
        ],
        output="log"
    )

    # ============================================================
    # FRAME COMPARISON
    # ============================================================

    frame_compare = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "drone_perception",
            "frame_compare"
        ],
        output="log"
    )

    # ============================================================
    # MOTION DETECTOR
    # ============================================================

    motion_detector = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "drone_perception",
            "motion_detector"
        ],
        output="log"
    )

    # ============================================================
    # PX4 MONITOR
    # ============================================================

    px4_monitor = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "drone_perception",
            "px4_monitor"
        ],
        output="log"
    )

    # ============================================================
    # QGROUNDCONTROL
    # ============================================================

    qgroundcontrol = ExecuteProcess(
        cmd=[
            "/home/vansh/Downloads/QGroundControl-x86_64.AppImage"
        ],
        output="log"
    )

    # ============================================================
    # LAUNCH
    # ============================================================

    return LaunchDescription([
        bridge,
        camera_tf,

        obstacle_detector,

        radar_receiver,
        radar_fusion,
        avoidance_controller,

        frame_processor,
        frame_compare,
        motion_detector,
        px4_monitor,

        qgroundcontrol,
    ])
