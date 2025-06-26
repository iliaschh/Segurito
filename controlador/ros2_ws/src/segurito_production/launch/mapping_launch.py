from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

PKG      = "segurito_production"
NAV2_PKG = "nav2_bringup"
EXPL_PKG = "nav2_wfd"
"""
LAUNCH MASTER
"""
def generate_launch_description():
    # rutas una vez construido el paquete ros2
    pkg_share   = get_package_share_directory(PKG)
    nav2_params = os.path.join(pkg_share, "config", "nav2_params.yaml") 
    slam_params = os.path.join(pkg_share, "config", "slam_mapping.yaml")

    # nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(NAV2_PKG),
                         "launch", "bringup_launch.py")),
        launch_arguments={
            "slam":             "True",
            "slam_params_file": slam_params,
            "map":              os.path.join(pkg_share,"config","dummy_map.yaml"),
            "autostart":        "True",
            "params_file":      nav2_params,
            "use_sim_time":     "False",
        }.items()
    )

    cm_node = ComposableNode(
        package='nav2_collision_monitor',
        plugin='nav2_collision_monitor::CollisionMonitor',
        name='collision_monitor',
        parameters=[nav2_params],         
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )

    load_cm = LoadComposableNodes(
        target_container='/nav2_container',     
        composable_node_descriptions=[cm_node]
    )
    # explorer
    explorer = Node(
        package=EXPL_PKG, executable="explore",
        parameters=[nav2_params],
        output="screen")
    # detección de personas
    people_alert = Node(
        package="segurito_production",
        executable="people_alert",
        output="screen",
    )

    # pausar nav2 si se detecta una persona
    person_stop = Node(
        package="segurito_production",
        executable="person_stop",
        output="screen",
    )

    
    
    # gestiona los modos del nav2 (generar mapa o cargar mapa)
    map_mode = Node(
        package="segurito_production",
        executable="map_mode_manager",
        output="screen")

    # guarda el mapa cuando explorer no detecta más fronteras
    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        output="screen",
        parameters=[nav2_params]
    )

    video_rec = Node(
        package  = "segurito_production",
        executable= "video_recorder",
        output   = "screen")
   

    return LaunchDescription([nav2, explorer, map_mode, map_saver, person_stop, people_alert])
