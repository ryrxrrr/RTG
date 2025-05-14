#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import time
import math
from copy import deepcopy

import rclpy
from rclpy.node import Node

import moveit_commander
from moveit_commander import MoveGroupCommander


class SwitchABDemo(Node):
    def __init__(self):
        super().__init__('switch_ab_demo')

        # 1) 初始化 moveit_commander 的后端 (等价于 ROS1 的 roscpp_initialize)
        moveit_commander.roscpp_initialize(sys.argv)

        # 2) 声明并获取一个参数 (相当于 ~cartesian)
        self.declare_parameter('cartesian', False)
        self.cartesian = self.get_parameter('cartesian').value
        self.get_logger().info(f"cartesian param = {self.cartesian}")

        # 3) 初始化 MoveGroupCommander
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

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

        # 获取终端link名称
        self.end_effector_link = self.arm.get_end_effector_link()

        # 机械臂先回到初始位置 home
        self.arm.set_named_target('home')
        self.arm.go()
        time.sleep(1)

        # 主循环逻辑：只要节点未关闭，就执行 pick & place
        while rclpy.ok():
            # to A
            if not rclpy.ok():
                break
            self.move2pose_eular(2, 0.05, 0.15, 0.17, math.pi / 2, 0, 0)
            self.gripper1.set_joint_value_target([0, 0])
            self.gripper1.go()
            time.sleep(2)

            # pick up A
            if not rclpy.ok():
                break
            self.move2pose_eular(1, 0.05, 0.15, 0.09, math.pi / 2, 0, 0)
            self.gripper1.set_joint_value_target([-0.022, -0.022])
            self.gripper1.go()
            time.sleep(2)

            if not rclpy.ok():
                break
            self.move2pose_eular(1, 0.05, 0.15, 0.17, math.pi / 2, 0, 0)

            # to tmp
            if not rclpy.ok():
                break
            self.move2pose_eular(2, 0.18, 0.0, 0.17, math.pi / 2, 0, 0)

            # putdown A
            if not rclpy.ok():
                break
            self.move2pose_eular(1, 0.18, 0.0, 0.10, math.pi / 2, 0, 0)
            self.gripper1.set_joint_value_target([0, 0])
            self.gripper1.go()
            time.sleep(2)

            if not rclpy.ok():
                break
            self.move2pose_eular(1, 0.18, 0.0, 0.17, math.pi / 2, 0, 0)

            # to B
            if not rclpy.ok():
                break
            self.move2pose_eular(2, 0.05, -0.15, 0.17, math.pi / 2, 0, 0)

            # pick up B
            if not rclpy.ok():
                break
            self.move2pose_eular(1, 0.05, -0.15, 0.09, math.pi / 2, 0, 0)
            self.gripper1.set_joint_value_target([-0.022, -0.022])
            self.gripper1.go()
            time.sleep(2)

            if not rclpy.ok():
                break
            self.move2pose_eular(1, 0.05, -0.15, 0.17, math.pi / 2, 0, 0)

            # to A
            if not rclpy.ok():
                break
            self.move2pose_eular(2, 0.05, 0.15, 0.17, math.pi / 2, 0, 0)

            # put down B
            if not rclpy.ok():
                break
            self.move2pose_eular(1, 0.05, 0.15, 0.10, math.pi / 2, 0, 0)
            self.gripper1.set_joint_value_target([0, 0])
            self.gripper1.go()
            time.sleep(2)

            if not rclpy.ok():
                break
            self.move2pose_eular(1, 0.05, 0.15, 0.17, math.pi / 2, 0, 0)

            # to tmp
            if not rclpy.ok():
                break
            self.move2pose_eular(2, 0.18, 0.0, 0.17, math.pi / 2, 0, 0)

            # pick up A
            if not rclpy.ok():
                break
            self.move2pose_eular(1, 0.18, 0.0, 0.09, math.pi / 2, 0, 0)
            self.gripper1.set_joint_value_target([-0.022, -0.022])
            self.gripper1.go()
            time.sleep(2)

            if not rclpy.ok():
                break
            self.move2pose_eular(1, 0.18, 0.0, 0.17, math.pi / 2, 0, 0)

            # to B
            if not rclpy.ok():
                break
            self.move2pose_eular(2, 0.05, -0.15, 0.17, math.pi / 2, 0, 0)

            # put down B
            if not rclpy.ok():
                break
            self.move2pose_eular(1, 0.05, -0.15, 0.10, math.pi / 2, 0, 0)
            self.gripper1.set_joint_value_target([0, 0])
            self.gripper1.go()
            time.sleep(2)

            if not rclpy.ok():
                break
            self.move2pose_eular(1, 0.05, -0.15, 0.17, math.pi / 2, 0, 0)

            # 一次完整交换后退出循环
            break

        # 回到 home, 然后 sleep
        self.arm.set_named_target('home')
        self.arm.go()
        time.sleep(1)
        self.arm.set_named_target('sleep')
        self.arm.go()
        time.sleep(1)

        # 最后松开夹爪
        self.gripper1.set_joint_value_target([0, 0])
        self.gripper1.go()
        time.sleep(0.5)

        # 关闭 MoveIt
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def move2pose_eular(self, times, px, py, pz, pitch, yaw, roll):
        """
        移动末端到 (px, py, pz) 并使用欧拉角 (pitch, yaw, roll) 计算出四元数。
        如果 cartesian=True，则把当前位置和目标位置加入路点，但并未真正调用 compute_cartesian_path()。
        如果 cartesian=False，则直接 set_pose_target & go。
        """
        # 获取当前位姿
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
        self.get_logger().info(f"Current pose: {start_pose}")

        # 生成目标位姿
        wpose = deepcopy(start_pose)
        # 代码里写了 wpose.position.z -= 0.25，但看你需求是否真的要先往下移 0.25
        wpose.position.x = px
        wpose.position.y = py
        wpose.position.z = pz

        # 欧拉角转四元数
        # pitch, yaw, roll -> x, y, z, w
        ox = (math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2) +
              math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2))
        oy = (math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2) +
              math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2))
        oz = (math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2) -
              math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2))
        ow = (math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2) -
              math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2))

        wpose.orientation.x = ox
        wpose.orientation.y = oy
        wpose.orientation.z = oz
        wpose.orientation.w = ow

        self.get_logger().info(f"Target pose:\n{wpose}")

        # 如果 cartesian=True, 仅仅把路点存储到列表，但没 compute_cartesian_path
        if self.cartesian:
            waypoints = [start_pose, deepcopy(wpose)]
            # 如果你想真的走笛卡尔插值，需要用:
            # plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0, True)
            # if fraction == 1.0:
            #     self.arm.execute(plan)
            # else:
            #     self.get_logger().warn(f"Cartesian path fraction={fraction}")
        else:
            # 否则, 自由曲线
            self.arm.set_pose_target(wpose)
            self.arm.go()

        # 等待 times 秒
        time.sleep(times)


def main(args=None):
    # 初始化 rclpy
    rclpy.init(args=args)

    # 创建并执行节点
    node = SwitchABDemo()

    # 退出
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
