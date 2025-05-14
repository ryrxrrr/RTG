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
        # 创建一个 ROS2 节点，名称为 'moveit_cartesian_demo'
        super().__init__('moveit_cartesian_demo')

        # 初始化 moveit_commander 的 C++ 后端 (类似 ROS1 的 roscpp_initialize)
        moveit_commander.roscpp_initialize(sys.argv)

        # 在 ROS2 中声明并获取参数(相当于 ~cartesian)
        self.declare_parameter('cartesian', True)
        cartesian = self.get_parameter('cartesian').value
        self.get_logger().info(f"cartesian param = {cartesian}")

        # 初始化需要使用 move group 控制的机械臂
        self.arm = MoveGroupCommander('arm', node=self)

        # 允许重新规划
        self.arm.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        self.arm.set_pose_reference_frame('sagittarius_arm_link')

        # 设置位置(单位：米)和姿态(单位：弧度)的允许误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()

        # ============ 以下为演示逻辑 ============ #

        # 控制机械臂先回到初始化位置 'home'
        self.arm.set_named_target('home')
        self.arm.go()
        time.sleep(1)

        # 获取当前位姿数据, 作为机械臂运动的起始位姿
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # 初始化路点列表
        waypoints = []

        # 如果为True, 将初始位姿加入路点列表
        if cartesian:
            waypoints.append(deepcopy(start_pose))

        # 依次设置几个目标位姿
        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.1
        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            self._go_directly_to_pose(wpose)

        wpose.position.x += 0.15
        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            self._go_directly_to_pose(wpose)

        wpose.position.y += 0.1
        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            self._go_directly_to_pose(wpose)

        wpose.position.x -= 0.15
        wpose.position.y -= 0.1
        if cartesian:
            waypoints.append(deepcopy(wpose))
        else:
            self._go_directly_to_pose(wpose)

        # 如果是笛卡尔规划(cartesian=True)，尝试 compute_cartesian_path
        if cartesian:
            fraction = 0.0    # 路径规划覆盖率
            maxtries = 100    # 最大尝试规划次数
            attempts = 0      # 已尝试次数

            # 设置当前的状态为运动初始状态
            self.arm.set_start_state_to_current_state()

            # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
            while fraction < 1.0 and attempts < maxtries and rclpy.ok():
                (plan, fraction) = self.arm.compute_cartesian_path(
                    waypoints,    # waypoint poses 路点列表
                    0.01,         # eef_step，终端步进值，每隔0.01m计算一次逆解
                    0.0,          # jump_threshold，跳跃阈值，0代表不允许跳跃
                    True          # avoid_collisions，是否进行避障
                )
                attempts += 1
                if attempts % 10 == 0:
                    self.get_logger().info(
                        f"Still trying after {attempts} attempts... (fraction={fraction})"
                    )

            # 如果路径规划成功（fraction=1.0），则执行
            if fraction == 1.0:
                self.get_logger().info("Path computed successfully. Moving the arm...")
                self.arm.execute(plan)
                self.get_logger().info("Path execution complete.")
            else:
                self.get_logger().info(
                    f"Path planning failed with only {fraction} success "
                    f"after {maxtries} attempts."
                )
            time.sleep(1)

        # 控制机械臂回到初始化位置 'home'
        self.arm.set_named_target('home')
        self.arm.go()
        time.sleep(1)

        # 关闭并退出 moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def _go_directly_to_pose(self, pose):
        """非笛卡尔模式下，直接 set_pose_target 并 go()。"""
        self.arm.set_pose_target(pose)
        self.arm.go()
        time.sleep(1)


def main(args=None):
    # 1) 初始化 rclpy
    rclpy.init(args=args)

    # 2) 创建并运行节点 (示例中直接执行逻辑，没用 spin)
    node = MoveItCartesianDemo()

    # 销毁节点并关闭 rclpy
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
