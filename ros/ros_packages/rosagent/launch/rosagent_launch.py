from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace="rosagent",
            package="rosagent",
            executable="rosagent_node",
            parameters=[
                {"ip_address": "127.0.0.1"},
                {"port": 7000},
                {"group_id": 0}
            ]
        )
    ])