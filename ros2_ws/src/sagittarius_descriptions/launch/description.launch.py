#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ConcatSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    sagittarius_descriptions_share = get_package_share_directory('sagittarius_descriptions')

    # 声明参数（替代<arg>）
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='sgr532',
        description='Name of the robot'
    )
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value=LaunchConfiguration('robot_name'),
        description='Robot model name (by default equal to robot_name)'
    )
    use_world_frame_arg = DeclareLaunchArgument(
        'use_world_frame', default_value='true',
        description='Whether to use a world link as root'
    )
    external_urdf_loc_arg = DeclareLaunchArgument(
        'external_urdf_loc', default_value='',
        description='External URDF location override'
    )
    load_gazebo_configs_arg = DeclareLaunchArgument(
        'load_gazebo_configs', default_value='false',
        description='Load Gazebo related configs if true'
    )
    joint_pub_gui_arg = DeclareLaunchArgument(
        'joint_pub_gui', default_value='true',
        description='Whether to launch the joint_state_publisher_gui'
    )
    use_joint_pub_arg = DeclareLaunchArgument(
        'use_joint_pub', default_value='false',
        description='Whether to use joint_state_publisher'
    )
    use_default_rviz_arg = DeclareLaunchArgument(
        'use_default_rviz', default_value='true',
        description='Whether to start RViz with default config'
    )
    rvizconfig_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=os.path.join(sagittarius_descriptions_share, 'rviz', 'description.rviz'),
        description='Path to the RViz config file'
    )

    # 模型文件路径(需根据robot_model动态生成)
    # 利用LaunchConfiguration与ConcatSubstitution构造出: <share_dir>/urdf/<robot_model>.urdf.xacro
    model_file = PathJoinSubstitution([
        FindPackageShare('sagittarius_descriptions'),
        'urdf',
        ConcatSubstitution([LaunchConfiguration('robot_model'), '.urdf.xacro'])
    ])

    # 声明model参数
    # 在ROS2中，xacro参数需通过Command传递，不再直接在default_value中写多行
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=model_file,
        description='Path to the URDF/Xacro file'
    )

    # 将xacro参数通过Command执行时传入
    # 相当于执行：
    # xacro --inorder <model_file> robot_name:=... use_world_frame:=... external_urdf_loc:=... load_gazebo_configs:=...
    robot_description = Command([
        'xacro', '--inorder', LaunchConfiguration('model'),
        ConcatSubstitution(['robot_name:=', LaunchConfiguration('robot_name')]),
        ConcatSubstitution(['use_world_frame:=', LaunchConfiguration('use_world_frame')]),
        ConcatSubstitution(['external_urdf_loc:=', LaunchConfiguration('external_urdf_loc')]),
        ConcatSubstitution(['load_gazebo_configs:=', LaunchConfiguration('load_gazebo_configs')])
    ])

    # robot_state_publisher节点
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # joint_state_publisher_gui节点（条件启动）
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[{'source_list': ['sagittarius_joint_states']}],
        condition=IfCondition(LaunchConfiguration('joint_pub_gui'))
    )

    # joint_state_publisher节点（条件启动）
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        output='screen',
        parameters=[{'source_list': ['sagittarius_joint_states']}],
        condition=IfCondition(LaunchConfiguration('use_joint_pub'))
    )

    # RViz节点（条件启动）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=LaunchConfiguration('robot_name'),
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_default_rviz'))
    )

    return LaunchDescription([
        robot_name_arg,
        robot_model_arg,
        use_world_frame_arg,
        external_urdf_loc_arg,
        load_gazebo_configs_arg,
        joint_pub_gui_arg,
        use_joint_pub_arg,
        use_default_rviz_arg,
        rvizconfig_arg,
        model_arg,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rsp_node,
        rviz_node
    ])
