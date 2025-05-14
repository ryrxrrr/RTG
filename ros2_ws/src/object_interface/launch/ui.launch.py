# ui.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_interface',
            executable='ui_node',
            name='ui_node',
            output='screen'
        )
    ])
