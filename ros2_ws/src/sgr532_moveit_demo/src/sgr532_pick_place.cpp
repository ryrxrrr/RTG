#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath> // for M_PI

/**
 * @brief 将角度(度)转换为弧度的简单助手函数
 */
static double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

/**
 * @brief 该类用来演示基于关节角度或基于末端Pose的两种运动方式
 */
class SGR532PickPlace
{
public:
  /**
   * @brief 构造函数：接收一个已经创建好的 rclcpp::Node::SharedPtr
   */
  explicit SGR532PickPlace(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  {
    // 创建 MoveGroupInterface 时，传入同一个 Node 指针即可
    move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
    move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "gripper");

    // 设置机械臂的规划参数
    move_group_arm_->setPlanningTime(5.0);
    move_group_arm_->setNumPlanningAttempts(10);
    move_group_arm_->setMaxVelocityScalingFactor(0.1);
    move_group_arm_->setMaxAccelerationScalingFactor(0.1);

    // 末端执行器链接（可根据实际 URDF 调整）
    move_group_arm_->setEndEffectorLink("sgr532/link6");

    RCLCPP_INFO(node_->get_logger(), "Reference frame: %s", move_group_arm_->getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", move_group_arm_->getEndEffectorLink().c_str());
  }

  /**
   * @brief 示例1：基于末端Pose的移动（你原先的写法）
   */
  bool moveToTarget(const geometry_msgs::msg::Pose& target_pose)
  {
    // 每次规划前，将起始状态设为当前状态，避免多次连续规划时状态不同步
    move_group_arm_->setStartStateToCurrentState();

    move_group_arm_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_arm_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(node_->get_logger(), "Planning successful!");
      auto error_code = move_group_arm_->execute(my_plan);
      if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(node_->get_logger(), "Execution successful!");
        return true;
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Execution failed with error code: %d", error_code.val);
        return false;
      }
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
      return false;
    }
  }

  /**
   * @brief 示例2：控制夹爪开合（如果仅仅做扫描，不一定用得上）
   * @param width 期望的夹爪宽度(两爪之间的距离)
   */
  bool moveGripper(double width)
  {
    try
    {
      // 获取夹爪组内的关节名称
      std::vector<std::string> joint_names = move_group_gripper_->getJointNames();
      if (joint_names.size() < 2)
      {
        RCLCPP_ERROR(node_->get_logger(), "Gripper joint names are not configured correctly!");
        return false;
      }

      // 设定目标位置（这里的逻辑是: 两个爪子分别 = -width/2）
      std::vector<double> joint_positions;
      joint_positions.push_back(-width / 2.0);
      joint_positions.push_back(-width / 2.0);

      RCLCPP_INFO(node_->get_logger(), "Joint names: %s, %s",
                  joint_names[0].c_str(), joint_names[1].c_str());
      RCLCPP_INFO(node_->get_logger(), "Joint positions: %f, %f",
                  joint_positions[0], joint_positions[1]);

      // 设置关节目标
      move_group_gripper_->setJointValueTarget(joint_names[0], joint_positions[0]);
      move_group_gripper_->setJointValueTarget(joint_names[1], joint_positions[1]);

      // 设置较短的规划时间
      move_group_gripper_->setPlanningTime(1.0);

      // 执行
      bool success = (move_group_gripper_->move() == moveit::core::MoveItErrorCode::SUCCESS);
      if (success)
      {
        RCLCPP_INFO(node_->get_logger(), "Gripper move successful!");
        return true;
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Gripper move failed!");
        return false;
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Exception in moveGripper: %s", e.what());
      return false;
    }
  }

  /**
   * @brief 示例3：基于末端Pose的多姿态扫描（你原先的逻辑）
   */
  bool performPickPlace()
  {
    // ======== 定义若干扫描姿态 ========
    geometry_msgs::msg::Pose pose_high;
    pose_high.position.x = 0.0;
    pose_high.position.y = 0.0;
    pose_high.position.z = 0.4;
    {
      tf2::Quaternion q;
      // roll=0, pitch=-30°, yaw=0
      q.setRPY(0.0, -M_PI/6, 0.0);
      pose_high.orientation = tf2::toMsg(q);
    }

    geometry_msgs::msg::Pose pose_high_left = pose_high;
    {
      tf2::Quaternion q;
      q.setRPY(0.0, -M_PI/6, M_PI/2); // yaw= +90
      pose_high_left.orientation = tf2::toMsg(q);
    }

    geometry_msgs::msg::Pose pose_high_right = pose_high;
    {
      tf2::Quaternion q;
      q.setRPY(0.0, -M_PI/6, -M_PI/2); // yaw= -90
      pose_high_right.orientation = tf2::toMsg(q);
    }

    geometry_msgs::msg::Pose pose_low_front;
    pose_low_front.position.x = 0.3;
    pose_low_front.position.y = 0.0;
    pose_low_front.position.z = 0.2;
    pose_low_front.orientation.x = 0.0;
    pose_low_front.orientation.y = 0.0;
    pose_low_front.orientation.z = 0.0;
    pose_low_front.orientation.w = 1.0;

    geometry_msgs::msg::Pose pose_low_left = pose_low_front;
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, M_PI/4); // yaw= +45
      pose_low_left.orientation = tf2::toMsg(q);
    }

    geometry_msgs::msg::Pose pose_low_right = pose_low_front;
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, -M_PI/4); // yaw= -45
      pose_low_right.orientation = tf2::toMsg(q);
    }

    // ======== 依次执行移动（模拟扫描） ========
    RCLCPP_INFO(node_->get_logger(), "Step 1: Move to high center (pitch=-30)");
    if (!moveToTarget(pose_high)) return false;

    RCLCPP_INFO(node_->get_logger(), "Step 2: Turn left (high, pitch=-30)");
    if (!moveToTarget(pose_high_left)) return false;

    RCLCPP_INFO(node_->get_logger(), "Step 3: Turn right (high, pitch=-30)");
    if (!moveToTarget(pose_high_right)) return false;

    RCLCPP_INFO(node_->get_logger(), "Step 4: Move to low front (pitch=0)");
    if (!moveToTarget(pose_low_front)) return false;

    RCLCPP_INFO(node_->get_logger(), "Step 5: Turn left (low)");
    if (!moveToTarget(pose_low_left)) return false;

    RCLCPP_INFO(node_->get_logger(), "Step 6: Turn right (low)");
    if (!moveToTarget(pose_low_right)) return false;

    RCLCPP_INFO(node_->get_logger(), "Return to high center (pitch=-30)");
    if (!moveToTarget(pose_high)) return false;

    RCLCPP_INFO(node_->get_logger(), "Scanning sequence completed successfully!");
    return true;
  }

  /**
   * @brief 示例4：基于关节角度做扫描 (低位 / 底座位 / 中位 / 高位)
   *
   * - 低位(Low): 
   *   - joint2=75°, joint3=-75°, joint4=90°, joint6=90°
   *   - joint1: -110°->110°, joint5: -95°->95°
   * - 底座位(Base):
   *   - joint2=20°, joint3=-5°, joint4=0°, joint6=-90°
   *   - joint1: -110°->110°, joint5=-100° (固定)
   * - 中位(Middle):
   *   - joint2=0°, joint3=60°, joint4=0°, joint6=0°
   *   - joint1: -110°->110°, joint5=-100° (固定)
   * - 高位(High):
   *   - joint1=0°, joint2=0°, joint3=90°, joint6=0°
   *   - joint4: -150°->150°, joint5=-100° (或也扫描)
   *
   * 注意：需要确认你的joint命名和索引是否匹配。
   */
  bool performJointSpaceMotion()
  {
    // 获取机械臂的所有关节名称
    std::vector<std::string> joint_names = move_group_arm_->getJointNames();
    if (joint_names.size() < 6)
    {
      RCLCPP_ERROR(node_->get_logger(), "Detected less than 6 joints. Check your MoveIt setup!");
      return false;
    }

    // 打印出MoveIt检测到的关节顺序
    RCLCPP_INFO(node_->get_logger(), "Arm joint order in MoveGroup:");
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(node_->get_logger(), "  [%lu] %s", i, joint_names[i].c_str());
    }

    // 用于临时存储目标关节角（度数）
    std::vector<double> joint_targets(6, 0.0);

    // 帮助函数：直接给move_group一个关节角目标（度数），并执行运动
    auto moveToJoints = [&](const std::vector<double>& target_degs) -> bool
    {
      // 把度数转成弧度并给 MoveIt
      for (size_t i = 0; i < target_degs.size(); ++i)
      {
        double rad = deg2rad(target_degs[i]);
        move_group_arm_->setJointValueTarget(joint_names[i], rad);
      }

      move_group_arm_->setStartStateToCurrentState();
      moveit::core::MoveItErrorCode err = move_group_arm_->move();
      if (err != moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "moveToJoints() failed. Code=%d", err.val);
        return false;
      }
      return true;
    };

    // ============ 低位 Low ============
    // joint2=75, joint3=-75, joint4=90, joint6=90
    // joint1(-110->110), joint5(-95->95)
    RCLCPP_INFO(node_->get_logger(), "===== Scanning Low position =====");
    joint_targets[1] = 75;    // joint2
    joint_targets[2] = -75;   // joint3
    joint_targets[3] = 90;    // joint4
    joint_targets[5] = 90;    // joint6

    for (double j1_deg = -110; j1_deg <= 110; j1_deg += 55.0)  // 每55度一个步进，仅供示例
    {
      for (double j5_deg = -95; j5_deg <= 95; j5_deg += 95.0) // 每95度一个步进，仅供示例
      {
        joint_targets[0] = j1_deg;  // joint1
        joint_targets[4] = j5_deg;  // joint5

        RCLCPP_INFO(node_->get_logger(),
                    "Low => j1=%.1f, j2=%.1f, j3=%.1f, j4=%.1f, j5=%.1f, j6=%.1f",
                    j1_deg, 75.0, -75.0, 90.0, j5_deg, 90.0);

        if (!moveToJoints(joint_targets)) {return false;}
      }
    }

    // ============ 底座位 Base ============
    // joint2=20, joint3=-5, joint4=0, joint6=-90
    // joint1(-110->110), joint5=-100 (固定)
    RCLCPP_INFO(node_->get_logger(), "===== Scanning Base position =====");
    joint_targets[1] = 20;    // joint2
    joint_targets[2] = -5;    // joint3
    joint_targets[3] = 0;     // joint4
    joint_targets[4] = -100;  // joint5 (固定)
    joint_targets[5] = -90;   // joint6

    for (double j1_deg = -110; j1_deg <= 110; j1_deg += 55.0)
    {
      joint_targets[0] = j1_deg; // joint1

      RCLCPP_INFO(node_->get_logger(),
                  "Base => j1=%.1f, j2=%.1f, j3=%.1f, j4=%.1f, j5=%.1f, j6=%.1f",
                  j1_deg, 20.0, -5.0, 0.0, -100.0, -90.0);

      if (!moveToJoints(joint_targets)) {return false;}
    }

    // ============ 中位 Middle ============
    // joint2=0, joint3=60, joint4=0, joint6=0
    // joint1(-110->110), joint5=-100 (固定)
    RCLCPP_INFO(node_->get_logger(), "===== Scanning Middle position =====");
    joint_targets[1] = 0;    // joint2
    joint_targets[2] = 60;   // joint3
    joint_targets[3] = 0;    // joint4
    joint_targets[4] = -100; // joint5(固定)
    joint_targets[5] = 0;    // joint6

    for (double j1_deg = -110; j1_deg <= 110; j1_deg += 55.0)
    {
      joint_targets[0] = j1_deg; // joint1

      RCLCPP_INFO(node_->get_logger(),
                  "Middle => j1=%.1f, j2=%.1f, j3=%.1f, j4=%.1f, j5=%.1f, j6=%.1f",
                  j1_deg, 0.0, 60.0, 0.0, -100.0, 0.0);

      if (!moveToJoints(joint_targets)) {return false;}
    }

    // ============ 高位 High ============
    // joint1=0, joint2=0, joint3=90, joint6=0
    // joint4(-150->150), joint5=-100(或也扫描)
    RCLCPP_INFO(node_->get_logger(), "===== Scanning High position =====");
    joint_targets[0] = 0;    // joint1
    joint_targets[1] = 0;    // joint2
    joint_targets[2] = 90;   // joint3
    joint_targets[5] = 0;    // joint6
    // 先从 joint4=-150 开始扫描到 +150
    joint_targets[3] = -150; // joint4
    joint_targets[4] = -100; // joint5(固定在-100，这里仅供示例)

    for (double j4_deg = -150; j4_deg <= 150; j4_deg += 75.0)
    {
      joint_targets[3] = j4_deg;

      RCLCPP_INFO(node_->get_logger(),
                  "High => j1=%.1f, j2=%.1f, j3=%.1f, j4=%.1f, j5=%.1f, j6=%.1f",
                  0.0, 0.0, 90.0, j4_deg, -100.0, 0.0);

      if (!moveToJoints(joint_targets)) {return false;}
    }

    RCLCPP_INFO(node_->get_logger(), "Joint-space scanning sequence completed successfully!");
    return true;
  }

private:
  // Node 指针：用来做日志、与 MoveGroupInterface 交互等
  rclcpp::Node::SharedPtr node_;

  // 机械臂 & 夹爪对应的 MoveGroupInterface
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // 在 main 中创建一个真正的节点
  auto node = std::make_shared<rclcpp::Node>("sgr532_pick_place_node");

  // 将这个 node 传给自定义类
  auto pick_place = std::make_shared<SGR532PickPlace>(node);

  RCLCPP_INFO(node->get_logger(), "=== Starting scanning sequence... ===");

  // 你可以选择使用基于Pose的扫描：
  // bool success = pick_place->performPickPlace();

  // 或者使用基于关节角度的扫描：
  bool success = pick_place->performJointSpaceMotion();

  if (!success)
  {
    RCLCPP_ERROR(node->get_logger(), "Scanning sequence failed or was cancelled!");
  }

  rclcpp::shutdown();
  return success ? 0 : 1;
}