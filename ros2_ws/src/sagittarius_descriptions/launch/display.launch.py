#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包共享目录
    sagittarius_descriptions_share = get_package_share_directory('sagittarius_descriptions')

    # 声明 model 参数（如果需要从命令行传入模型路径）
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(sagittarius_descriptions_share, 'urdf', 'sgr532.urdf.xacro'),
        description='Path to the URDF file'
    )

    model = LaunchConfiguration('model')

    # RViz 配置文件路径
    rviz_config_file = os.path.join(sagittarius_descriptions_share, 'urdf.rviz')

    # 使用打开文件的方式加载 robot_description（适用于纯URDF文件，无需xacro处理）
    # ⚠ 如果你的文件实际上是 xacro 并含有宏，还需要先做 xacro 展开，否则只读原 xacro 源码可能会导致解析不完整。
    with open(os.path.join(sagittarius_descriptions_share, 'urdf', 'sgr532.urdf'), 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # 把加载后的字符串打包成 Python 字典，方便传给各节点
    robot_desc_param = {'robot_description': robot_description_content}

    # robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_desc_param],
        output='screen'
    )

    # joint_state_publisher_gui 节点 —— 这里也需要同样的 robot_description
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[robot_desc_param]
    )

    # RViz 节点（ROS2中使用rviz2可执行文件）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        # 顺序无所谓，但都要包含在 LaunchDescription 中
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
