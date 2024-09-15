import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kamera_tf2_py",
            executable="kamera_tf2",
            name="kamera_tf2"
        ),
        Node(
            package="kamera_tf2_py",
            executable="static_marker",
            name="static_marker"
        ),
        Node(
            package="kamera_tf2_py",
            executable="move_robot",
            name="move_robot"
        ),
    ])