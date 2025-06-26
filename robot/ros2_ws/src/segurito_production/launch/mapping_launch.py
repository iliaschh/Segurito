from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

PKG = "segurito_production"

def generate_launch_description():
    pkg_share = get_package_share_directory(PKG)

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rplidar_ros"),
                         "launch", "rplidar_c1_launch.py")),
        launch_arguments={
            "serial_port": "/dev/ttyUSB0",
            "scan_frequency": "8.0",
        }.items()
    )

    odom = Node(package=PKG, executable="odometria",
                parameters=[{"wheel_radius": 0.018,
                             "track_width": 0.117,
                             "ticks_per_rev": 900,
                             "period_ms": 50,
                             "publish_tf": True}])

    static_tf = Node(package=PKG, executable="static_laser_tf")

    drive = Node(
        package=PKG,
        executable="drive_base",
        parameters=[{
            "wheel_radius":   0.018,
            "track_width":    0.117,
            "max_speed_mps":  0.12,
            "left_trim":  1.0,   # izquierda algo más lenta la subo 5 %
            "right_trim": 1.0,
            "left_dir":  -1,      # invierto solo la rueda izda
            "right_dir":  -1
        }]
    )


    return LaunchDescription([rplidar, odom, static_tf, drive])
