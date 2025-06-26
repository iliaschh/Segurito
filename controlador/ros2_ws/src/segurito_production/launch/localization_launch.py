from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

PKG      = "segurito_production"
NAV2_PKG = "nav2_bringup"

def generate_launch_description():
    pkg_share   = get_package_share_directory(PKG)
    nav2_params = os.path.join(pkg_share, "config", "nav2_params.yaml")

    map_yaml_arg = DeclareLaunchArgument(
        "map",
        description="YAML del mapa a cargar (ruta absoluta)"
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(NAV2_PKG),
                         "launch", "bringup_launch.py")),
        launch_arguments={
            "slam":        "False",
            "map":         os.getenv("map"),      
            "autostart":   "True",
            "params_file": nav2_params,
            "use_sim_time":"False",
        }.items()
    )

    people_alert = Node(
        package=PKG, executable="people_alert",  output="screen")

    person_stop  = Node(
        package=PKG, executable="person_stop",   output="screen")

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="patroller",
        output="screen",
        parameters=[nav2_params],
    )

    return LaunchDescription([
        map_yaml_arg,
        nav2,
        people_alert,
        person_stop,
        waypoint_follower
    ])
