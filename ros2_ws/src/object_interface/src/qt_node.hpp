#ifndef OBJECT_INTERFACE__QT_NODE_HPP_
#define OBJECT_INTERFACE__QT_NODE_HPP_

#include <QWidget>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTimer>  // 定时器

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <my_bridge_pkg/msg/grasp_point.hpp>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

// C++ STL
#include <unordered_map>
#include <string>
#include <memory>
#include <mutex>

/**
 * @brief 单个抓取信息
 */
struct GraspData
{
  float x;
  float y;
  float z;
  float angle;      // 弧度 (绕Z轴 或者绕X轴等，视情况而定)
  float width;      // 识别到的爪子宽度
  float score;
  float confidence;
};

/**
 * @brief QtNode
 *
 *  - 点击 Pick: 
 *      - 禁用碰撞、暂停环境更新
 *      - 机械臂到达抓取点 => 沿X轴再前进6cm => 爪子张开/收紧
 *  - 点击 PickWA: 与上类似，但忽略angle(固定抓取角度)
 *  - 点击 Deliver: 
 *      - 先获取当前关节角度，让 joint2+=15度 
 *      - 再执行原先的递送角度
 *  - 点击 Release: 爪子放宽一定量，恢复碰撞检测、恢复环境更新
 *  - 点击 Clear: 清空抓取目标
 *  - 点击 Choose: 测试让末端移动到(x, y, z+0.05)
 *  - 点击 Env: 可执行外部命令进行扫描等操作
 *
 *  - 爪子关节范围: [-0.034, 0.0]（依据具体机械臂夹爪而定）
 */
class QtNode : public QWidget, public rclcpp::Node
{
    Q_OBJECT
public:
    explicit QtNode(const std::string & node_name, QWidget *parent = nullptr);

    // 构造结束后再调用，用于初始化 MoveIt
    void initMoveIt();

    // 显示 Qt 窗口
    void show();

private slots:
    void onPickClicked();
    void onDeliverClicked();
    void onClearClicked();
    void onReleaseClicked();

    // 定时器的槽函数：安全刷新 UI
    void updateUI();

    // Choose 按钮槽函数
    void onChooseClicked();

    // “PickWA” 按钮槽函数 (忽略angle抓取)
    void onPickWAClicked();

    // “Env” 按钮槽函数
    void onEnvClicked();

private:
    // ---- ROS 回调 ----
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void annotatedImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void graspCallback(const my_bridge_pkg::msg::GraspPoint::SharedPtr msg);

    // ---- MoveIt相关 ----
    bool moveArmToPose(float x, float y, float z, float yaw_rad);
    bool moveArmToJoints(const std::vector<double> &joint_values);

    bool moveGripper(double width);

    /**
     * @brief 在当前末端姿态的基础上，沿 X/Y/Z 做一个相对位移（单位: 米）。
     *
     * @param dx X方向增量
     * @param dy Y方向增量
     * @param dz Z方向增量
     * @return true 成功
     * @return false 失败
     */
    bool moveArmRelativeOffset(double dx, double dy, double dz);

    // ==== 碰撞检测的启停 ====
    void disableAllCollisions();
    void enableAllCollisionsBack();

    // ==== 暂停/恢复环境建模 ====
    void stopEnvironmentUpdate();
    void resumeEnvironmentUpdate();

    // ---- UI控件 ----
    QLabel* rgb_label_;
    QLabel* bottom_label_;
    QListWidget* object_list_;

    QPushButton* pick_btn_;
    QPushButton* deliver_btn_;
    QPushButton* clear_btn_;
    QPushButton* release_btn_;

    QPushButton* choose_btn_;
    QPushButton* pick_wa_btn_;
    QPushButton* env_btn_;

    QHBoxLayout* main_layout_;
    QVBoxLayout* left_layout_;
    QVBoxLayout* right_layout_;

    // ---- 订阅者 ----
    rclcpp::Subscription<my_bridge_pkg::msg::GraspPoint>::SharedPtr grasp_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr annotated_image_sub_;

    // 存储抓取数据
    std::unordered_map<std::string, GraspData> label2grasp_;
    std::mutex data_mutex_;

    // ---- MoveGroup接口 (机械臂 + 夹爪) ----
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_;

    // ---- PlanningSceneMonitor ----
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    // ---- ApplyPlanningScene 服务客户端 ----
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;
};

#endif  // OBJECT_INTERFACE__QT_NODE_HPP_
