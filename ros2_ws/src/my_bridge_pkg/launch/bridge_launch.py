# bridge_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_bridge_pkg',
            executable='bridge_node',
            name='bridge_node'
        ),
        # 也可以在这里添加 YOLO 节点 etc.
    ])
