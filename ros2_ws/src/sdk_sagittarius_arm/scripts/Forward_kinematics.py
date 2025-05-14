#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import time

import rclpy
from rclpy.node import Node

import moveit_commander
from moveit_commander import MoveGroupCommander

def main(args=None):
    # 1) 初始化 rclpy
    rclpy.init(args=args)

    # 2) 初始化 moveit_commander 的 C++ 后端 (与roscpp对应)
    moveit_commander.roscpp_initialize(sys.argv)

    # 3) 创建 ROS2 Node
    node = rclpy.create_node('moveit_cartesian_demo')

    # 等价于原先的 ~cartesian 参数，如果在命令行想改： 
    #   ros2 run your_package forward_kinematics.py --ros-args -p cartesian:=true
    node.declare_parameter('cartesian', False)
    cartesian = node.get_parameter('cartesian').value  # 默认为 False

    # 4) 创建 MoveGroupCommander
    arm = MoveGroupCommander('sagittarius_arm', node=node)
    gripper = MoveGroupCommander('sagittarius_gripper', node=node)
    gripper1 = MoveGroupCommander('sagittarius_gripper', node=node)

    # 允许重新规划
    arm.allow_replanning(True)

    # 设置参考坐标系
    arm.set_pose_reference_frame('world')
    gripper.set_pose_reference_frame('world')

    # 设置位置/姿态误差阈值
    arm.set_goal_position_tolerance(0.0001)
    arm.set_goal_orientation_tolerance(0.0001)

    gripper1.set_goal_joint_tolerance(0.001)

    # 设置允许的最大速度和加速度
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_max_velocity_scaling_factor(0.5)

    end_effector_link = arm.get_end_effector_link()

    while rclpy.ok():
        node.get_logger().info("set joint 2: 0° -> 45°, joint 3: 0° -> -45°")
        tjoint = [0, math.pi/4, -math.pi/4, 0, 0, 0]
        arm.set_joint_value_target(tjoint)
        arm.go()
        time.sleep(0.5)
        if not rclpy.ok():
            break
        node.get_logger().info(f"Current pose:\n{arm.get_current_pose(end_effector_link).pose}")

        node.get_logger().info("set joint 4: 0° -> 90°, joint 6: 0° -> 90°")
        tjoint = [0, math.pi/4, -math.pi/4, math.pi/2, 0, math.pi/2]
        arm.set_joint_value_target(tjoint)
        arm.go()
        time.sleep(0.5)
        if not rclpy.ok():
            break
        node.get_logger().info(f"Current pose:\n{arm.get_current_pose(end_effector_link).pose}")

        node.get_logger().info("set joint 4: 90° -> -90°, joint 6: 90° -> -90°")
        tjoint = [0, math.pi/4, -math.pi/4, -math.pi/2, 0, -math.pi/2]
        arm.set_joint_value_target(tjoint)
        arm.go()
        time.sleep(0.5)
        if not rclpy.ok():
            break
        node.get_logger().info(f"Current pose:\n{arm.get_current_pose(end_effector_link).pose}")

        node.get_logger().info("set joint 3: 0° -> 90°, joint 5: 0° -> -90°")
        tjoint = [0, 0, math.pi/2, 0, -math.pi/2, 0]
        arm.set_joint_value_target(tjoint)
        arm.go()
        time.sleep(0.5)
        if not rclpy.ok():
            break
        node.get_logger().info(f"Current pose:\n{arm.get_current_pose(end_effector_link).pose}")

        node.get_logger().info("set joint 4: 0° -> 45°")
        tjoint = [0, 0, math.pi/2, math.pi/4, -math.pi/2, 0]
        arm.set_joint_value_target(tjoint)
        arm.go()
        time.sleep(0.5)
        if not rclpy.ok():
            break
        node.get_logger().info(f"Current pose:\n{arm.get_current_pose(end_effector_link).pose}")

        node.get_logger().info("set joint 4: 45° -> -45°")
        tjoint = [0, 0, math.pi/2, -math.pi/4, -math.pi/2, 0]
        arm.set_joint_value_target(tjoint)
        arm.go()
        time.sleep(0.5)
        if not rclpy.ok():
            break
        node.get_logger().info(f"Current pose:\n{arm.get_current_pose(end_effector_link).pose}")

        # 做一次演示后，退出循环
        break

    # 回到 home 与 sleep
    arm.set_named_target('home')
    arm.go()
    time.sleep(1)
    arm.set_named_target('sleep')
    arm.go()
    time.sleep(1)

    gripper1.set_joint_value_target([0, 0])
    gripper1.go()
    time.sleep(0.5)

    # 5) 关闭 moveit_commander 和节点
    moveit_commander.roscpp_shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
