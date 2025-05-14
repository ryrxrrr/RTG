#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import sys
import time

import rclpy
from rclpy.node import Node

import moveit_commander
from moveit_commander import MoveGroupCommander

class MoveItCartesianDemo(Node):
    def __init__(self):
        # 创建一个名为 'moveit_cartesian_demo' 的 ROS2 节点
        super().__init__('moveit_cartesian_demo')

        # 初始化 moveit_commander 的 C++ 后端 (相当于 roscpp_initialize)
        moveit_commander.roscpp_initialize(sys.argv)

        # 在 ROS2 中声明并获取参数（相当于 ~cartesian）
        self.declare_parameter('cartesian', False)
        cartesian = self.get_parameter('cartesian').value
        self.get_logger().info(f"cartesian param: {cartesian}")

        # 初始化需要使用 move group 控制的机械臂和夹爪
        arm = MoveGroupCommander('arm', node=self)
        gripper = MoveGroupCommander('gripper', node=self)
        gripper1 = MoveGroupCommander('gripper', node=self)

        # 允许重新规划
        arm.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('sagittarius_arm_link')
        gripper.set_pose_reference_frame('sagittarius_arm_link')

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        gripper1.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # 获取末端执行器的 link 名称
        end_effector_link = arm.get_end_effector_link()

        # 循环获取当前位姿（示例：打印到屏幕）
        self.get_logger().info("Start printing current pose. Press Ctrl+C to stop.")
        while rclpy.ok():
            current_pose = arm.get_current_pose(end_effector_link).pose
            self.get_logger().info(f"Current pose:\n{current_pose}")
            time.sleep(0.5)  # 或者使用 rclpy.spin_once(self, timeout_sec=0.5)

        # 循环被中断（Ctrl+C）后，退出前关闭 MoveIt
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


def main(args=None):
    # 1) 初始化 rclpy
    rclpy.init(args=args)

    # 2) 创建并运行节点
    node = MoveItCartesianDemo()
    
    try:
        # 如果脚本中需要 spin，就在这里spin
        # 不过上面用 while rclpy.ok() + time.sleep(...) 也可以
        # rclpy.spin(node)
        pass
    except KeyboardInterrupt:
        pass
    finally:
        # 正常退出
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
