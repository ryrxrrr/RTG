#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import time
from copy import deepcopy

import rclpy
from rclpy.node import Node

import moveit_commander
from moveit_commander import MoveGroupCommander


class MoveItCartesianDemo(Node):
    def __init__(self):
        super().__init__('moveit_cartesian_demo')

        # 初始化 moveit_commander 的 C++ 后端 (相当于 ROS1 的 roscpp_initialize)
        moveit_commander.roscpp_initialize(sys.argv)

        # 声明并获取一个参数 (相当于 ~cartesian)
        self.declare_parameter('cartesian', False)
        self.cartesian = self.get_parameter('cartesian').value
        self.get_logger().info(f"cartesian param = {self.cartesian}")

        # 初始化 move group 控制的 arm group 与 gripper group
        self.arm = MoveGroupCommander('arm', node=self)
        self.gripper = MoveGroupCommander('gripper', node=self)
        self.gripper1 = MoveGroupCommander('gripper', node=self)

        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        self.arm.set_pose_reference_frame('world')
        self.gripper.set_pose_reference_frame('world')

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
        self.gripper1.set_goal_joint_tolerance(0.001)

        # 设置最大速度、加速度比例
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

        # 获取末端执行器 link 名
        self.end_effector_link = self.arm.get_end_effector_link()

        # 机械臂回到初始位置 'home'
        self.arm.set_named_target('home')
        self.arm.go()
        time.sleep(1)

        # 夹爪：合拢一点
        self.get_logger().info("Gripper close a bit...")
        self.gripper1.set_joint_value_target([-0.005, -0.005])
        self.gripper1.go()
        time.sleep(1)

        # 在循环中调用 move2pose() 做一些位姿移动
        self.get_logger().info("Start moving in a loop. Press Ctrl+C to stop.")
        while rclpy.ok():
            # 第一步
            self.move2pose(1.0, 0.15, -0.1, 0.25, 0.0, 0.0, 0.0, 1.0)
            if not rclpy.ok():
                break
            # 第二步
            self.move2pose(1.0, 0.25, -0.1, 0.25, 0.0, 0.0, 0.0, 1.0)
            if not rclpy.ok():
                break

            # 演示一次后退出循环
            break

        # 回到 home -> sleep
        self.arm.set_named_target('home')
        self.arm.go()
        time.sleep(1)
        self.arm.set_named_target('sleep')
        self.arm.go()
        time.sleep(1)

        # 关闭 MoveIt Commander
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def move2pose(self, wait_time, px, py, pz, ox, oy, oz, ow):
        """
        移动机械臂到指定的 (px, py, pz, ox, oy, oz, ow)。
        如果 cartesian=True，则可以做笛卡尔直线轨迹；否则用自由曲线。
        """
        # 获取当前位姿
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
        self.get_logger().info(f"Current pose before move:\n{start_pose}")

        # 初始化路点列表
        waypoints = []

        # 如果 cartesian=True，将初始位姿加入路点列表
        if self.cartesian:
            waypoints.append(start_pose)

        # 设置新的目标姿态
        wpose = deepcopy(start_pose)
        # 示例：先往下 z 减 0.25 (你原代码里这样写，但可根据实际需要修改)
        # wpose.position.z -= 0.25
        wpose.position.x = px
        wpose.position.y = py
        wpose.position.z = pz
        wpose.orientation.x = ox
        wpose.orientation.y = oy
        wpose.orientation.z = oz
        wpose.orientation.w = ow

        self.get_logger().info("Target pose:")
        self.get_logger().info(f"{wpose}")

        # 如果 cartesian=True，则直线轨迹
        if self.cartesian:
            waypoints.append(deepcopy(wpose))
            # 规划笛卡尔路径
            (plan, fraction) = self.arm.compute_cartesian_path(
                waypoints,   # 路点
                0.01,        # eef_step
                0.0,         # jump_threshold
                True         # avoid_collisions
            )
            if fraction == 1.0:
                self.get_logger().info("Cartesian path computed, executing...")
                self.arm.execute(plan)
            else:
                self.get_logger().warn(
                    f"Cartesian path fraction={fraction}, not fully successful."
                )
        else:
            # 如果不是笛卡尔轨迹，则直接 set_pose_target -> go
            self.arm.set_pose_target(wpose)
            self.arm.go()

        # 等待一段时间
        time.sleep(wait_time)


def main(args=None):
    # 1) 初始化 rclpy
    rclpy.init(args=args)

    # 2) 创建并执行节点
    node = MoveItCartesianDemo()

    # 关闭节点
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
