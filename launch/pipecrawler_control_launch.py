from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    server_node = Node(
        package="pipecrawler",
        executable="pipecrawler_server.py",
        output = 'screen'
    )

    ld.add_action(server_node)
    return ld