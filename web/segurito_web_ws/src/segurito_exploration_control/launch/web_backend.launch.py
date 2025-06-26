from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        parameters=[{"port": 9090}],
        output="screen",
    )

    explore = Node(
        package   ="segurito_exploration_control",
        executable="exploration_service",
        output    ="screen",
    )

    roboflow_srv = ExecuteProcess(
        name = "roboflow_server",
        cmd  = [
            "inference", "server", "start",
        ],
        output = "screen",
    )

    people = TimerAction(
        period=5.0,
        actions=[
            Node(
                package   ="segurito_exploration_control",
                executable="people_detector",
                output    ="screen",
            )
        ],
    )

    map_files = Node(
        package="segurito_exploration_control",
        executable="map_files_server",
        output="screen",
    )

    video_files = Node(
        package="segurito_exploration_control",
        executable="video_files_server",
        output="screen",
    )

    return LaunchDescription([rosbridge, map_files, video_files, people, roboflow_srv])
