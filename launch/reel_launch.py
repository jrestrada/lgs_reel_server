from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="reel",
            executable="reel_server.py",
            output="screen"
        ),
    ])