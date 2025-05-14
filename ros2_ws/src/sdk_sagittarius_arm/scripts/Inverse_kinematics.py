#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import math
import time
from copy import deepcopy

import rclpy
from rclpy.node import Node

import moveit_commander
from moveit_commander import MoveGroupCommander

def eular2orientation(pitch, yaw, roll):
    """将欧拉角(pitch, yaw, roll)转换为四元数(ox, oy, oz, ow)。"""
    ox = (math.sin(pitch/2)*math.sin(yaw/2)*math.cos(roll/2) +
          math.cos(pitch/2)*math.cos(yaw/2)*math.sin(roll/2))
    oy = (math.sin(pitch/2)*math.cos(yaw/2)*math.cos(roll/2) +
          math.cos(pitch/2)*math.sin(yaw/2)*math.sin(roll/2))
    oz = (math.cos(pitch/2)*math.sin(yaw/2)*math.cos(roll/2) -
          math.sin(pitch/2)*math.cos(yaw/2)*math.sin(roll/2))
    ow = (math.cos(pitch/2)*math.cos(yaw/2)*math.cos(roll/2) -
          math.sin(pitch/2)*math.sin(yaw/2)*math.sin(roll/2))
    return ox, oy, oz, ow

class InverseKinematicsDemo(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_demo')

        # 初始化 moveit_commander 的 C++ 后端 (与原先 roscpp_initialize 类似)
        moveit_commander.roscpp_initialize(sys.argv)

        # 在 ROS2 中声明并获取参数（~cartesian），默认值为 False
        self.declare_parameter('cartesian', False)
        cartesian = self.get_parameter('cartesian').value
        self.get_logger().info(f"cartesian param = {cartesian}")

        # 初始化需要使用 move group 控制的机械臂
        self.arm = MoveGroupCommander('sagittarius_arm', node=self)
        self.gripper = MoveGroupCommander('sagittarius_gripper', node=self)
        self.gripper1 = MoveGroupCommander('sagittarius_gripper', node=self)

        # 允许/不允许重新规划
        self.arm.allow_replanning(False)

        # 设置参考坐标系
        self.arm.set_pose_reference_frame('world')
        self.gripper.set_pose_reference_frame('world')

        # 设置容许误差
        self.arm.set_goal_position_tolerance(0.0001)
        self.arm.set_goal_orientation_tolerance(0.0001)
        self.gripper1.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

        # 获取末端执行器的 link
        self.end_effector_link = self.arm.get_end_effector_link()

    def move2pose(self, wait_time, px, py, pz, ox, oy, oz, ow):
        """移动机械臂到指定的 (位置 + 姿态)"""
        wpose = deepcopy(self.arm.get_current_pose(self.end_effector_link).pose)
        wpose.position.x = px
        wpose.position.y = py
        wpose.position.z = pz
        wpose.orientation.x = ox
        wpose.orientation.y = oy
        wpose.orientation.z = oz
        wpose.orientation.w = ow

        self.arm.set_pose_target(wpose)
        self.arm.go()
        time.sleep(wait_time)


def main(args=None):
    # 1) 初始化 rclpy
    rclpy.init(args=args)

    # 2) 创建节点对象
    node = InverseKinematicsDemo()

    # ------------------  以下部分是脚本的主要操作流程  ------------------ #

    # 让机械臂先回到 'home' 姿态
    node.arm.set_named_target('home')
    node.arm.go()
    time.sleep(1)

    # 获取当前末端位姿(作为起始点)
    pose = deepcopy(node.arm.get_current_pose(node.end_effector_link).pose)
    px = pose.position.x
    py = pose.position.y
    pz = pose.position.z
    ox = pose.orientation.x
    oy = pose.orientation.y
    oz = pose.orientation.z
    ow = pose.orientation.w

    # 模拟循环：每次在 X/Y/Z 上移动一点，构成若干个点的运动
    while rclpy.ok():
        px -= 0.1
        node.move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if not rclpy.ok():
            break

        py -= 0.1
        node.move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if not rclpy.ok():
            break

        pz -= 0.1
        node.move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if not rclpy.ok():
            break

        py += 0.20
        node.move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if not rclpy.ok():
            break

        pz += 0.1
        node.move2pose(0.5, px, py, pz, ox, oy, oz, ow)
        if not rclpy.ok():
            break

        # 演示一次后退出循环
        break

    # 回到 home 和 sleep 姿态
    node.arm.set_named_target('home')
    node.arm.go()
    time.sleep(1)
    node.arm.set_named_target('sleep')
    node.arm.go()
    time.sleep(1)

    # 夹爪复位
    node.gripper1.set_joint_value_target([0, 0])
    node.gripper1.go()
    time.sleep(0.5)

    # 关闭 moveit_commander
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

    # 销毁节点 & 关闭 rclpy
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
