#include "qt_node.hpp"

// Qt
#include <QLabel>
#include <QPushButton>
#include <QListWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTimer>
#include <QMessageBox>

// ROS2 + OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// TF
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// MoveIt
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

// C++
#include <cmath> // for M_PI
#include <mutex>
#include <chrono>
#include <thread>


QtNode::QtNode(const std::string &node_name, QWidget *parent)
    : QWidget(parent)
    , rclcpp::Node(node_name)
{
    RCLCPP_INFO(this->get_logger(), "QtNode constructor start...");

    // 1) UI
    rgb_label_ = new QLabel("RGB Image here", this);
    rgb_label_->setMinimumSize(320, 240);
    rgb_label_->setStyleSheet("QLabel { background-color : #cccccc; }");
    rgb_label_->setAlignment(Qt::AlignCenter);

    bottom_label_ = new QLabel("Annotated Image here", this);
    bottom_label_->setMinimumSize(320, 240);
    bottom_label_->setStyleSheet("QLabel { background-color : #eeeeee; }");
    bottom_label_->setAlignment(Qt::AlignCenter);

    object_list_ = new QListWidget(this);
    pick_btn_    = new QPushButton("Pick", this);
    deliver_btn_ = new QPushButton("Deliver", this);
    clear_btn_   = new QPushButton("Clear", this);
    release_btn_ = new QPushButton("Release", this);

    choose_btn_   = new QPushButton("Choose", this);
    pick_wa_btn_  = new QPushButton("PickWA", this);
    env_btn_      = new QPushButton("Env", this);

    main_layout_  = new QHBoxLayout(this);
    left_layout_  = new QVBoxLayout();
    right_layout_ = new QVBoxLayout();

    left_layout_->addWidget(rgb_label_);
    left_layout_->addWidget(bottom_label_);

    right_layout_->addWidget(object_list_);

    // 按钮区域
    QHBoxLayout* btn_layout = new QHBoxLayout();
    btn_layout->addWidget(pick_btn_);
    btn_layout->addWidget(deliver_btn_);
    btn_layout->addWidget(release_btn_);
    btn_layout->addWidget(choose_btn_);
    btn_layout->addWidget(pick_wa_btn_);
    right_layout_->addLayout(btn_layout);
    right_layout_->addWidget(clear_btn_);
    right_layout_->addWidget(env_btn_);

    main_layout_->addLayout(left_layout_, 1);
    main_layout_->addLayout(right_layout_, 0);
    setLayout(main_layout_);

    // 2) 信号槽
    connect(pick_btn_, &QPushButton::clicked, this, &QtNode::onPickClicked);
    connect(deliver_btn_, &QPushButton::clicked, this, &QtNode::onDeliverClicked);
    connect(clear_btn_, &QPushButton::clicked, this, &QtNode::onClearClicked);
    connect(release_btn_, &QPushButton::clicked, this, &QtNode::onReleaseClicked);
    connect(choose_btn_, &QPushButton::clicked, this, &QtNode::onChooseClicked);
    connect(pick_wa_btn_, &QPushButton::clicked, this, &QtNode::onPickWAClicked);
    connect(env_btn_, &QPushButton::clicked, this, &QtNode::onEnvClicked);

    // 3) 订阅
    grasp_sub_ = this->create_subscription<my_bridge_pkg::msg::GraspPoint>(
        "/bridge/world_grasp_point",
        10,
        std::bind(&QtNode::graspCallback, this, std::placeholders::_1)
    );

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_rect_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&QtNode::imageCallback, this, std::placeholders::_1)
    );

    annotated_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/annotated_image",
        rclcpp::SensorDataQoS(),
        std::bind(&QtNode::annotatedImageCallback, this, std::placeholders::_1)
    );

    // 4) 定时器
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &QtNode::updateUI);
    timer->start(200);

    RCLCPP_INFO(this->get_logger(), "QtNode constructor done.");
}

void QtNode::initMoveIt()
{
    RCLCPP_INFO(this->get_logger(), "initMoveIt() start...");
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    try
    {
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(),
            "arm",
            tf_buffer,
            rclcpp::Duration(5,0)
        );
        RCLCPP_INFO(this->get_logger(), "Created arm MoveGroupInterface. Setting end effector link=grasp_point_link...");
        arm_->setEndEffectorLink("grasp_point_link");

        gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(),
            "gripper",
            tf_buffer,
            rclcpp::Duration(5,0)
        );
        RCLCPP_INFO(this->get_logger(), "Created gripper MoveGroupInterface.");

        arm_->setPlanningTime(5.0);
        arm_->setNumPlanningAttempts(10);
        arm_->setMaxVelocityScalingFactor(0.2);
        arm_->setMaxAccelerationScalingFactor(0.2);

        RCLCPP_INFO(this->get_logger(),
            "Arm planning frame: %s", arm_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(),
            "Arm end effector link: %s", arm_->getEndEffectorLink().c_str());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),
            "Exception in creating MoveGroupInterface: %s", e.what());
        return;
    }

    // PlanningSceneMonitor
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        this->shared_from_this(),
        "robot_description",
        "planning_scene_monitor"
    );
    if(!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene())
    {
        RCLCPP_ERROR(this->get_logger(),
            "Failed to create PlanningSceneMonitor or getPlanningScene() is null!");
        return;
    }

    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");

    RCLCPP_INFO(this->get_logger(), "PlanningSceneMonitor started successfully.");

    // /apply_planning_scene 客户端
    apply_planning_scene_client_ =
        this->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");

    RCLCPP_INFO(this->get_logger(), "initMoveIt() done.");
}

void QtNode::show()
{
    RCLCPP_INFO(this->get_logger(), "QtNode::show() called => showing GUI...");
    QWidget::show();
    this->resize(800, 480);
}

//---------------------------------------
// Pick
//---------------------------------------
void QtNode::onPickClicked()
{
    auto item = object_list_->currentItem();
    if(!item)
    {
        RCLCPP_WARN(this->get_logger(), "No item selected in list => abort pick!");
        return;
    }
    std::string label = item->text().toStdString();

    GraspData gd;
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        if(!label2grasp_.count(label))
        {
            RCLCPP_WARN(this->get_logger(), "No grasp data for label=%s => abort pick!", label.c_str());
            return;
        }
        gd = label2grasp_[label];
    }

    RCLCPP_INFO(this->get_logger(),
        "[onPickClicked] (x=%.3f,y=%.3f,z=%.3f, angle=%.2f, width=%.3f)",
        gd.x, gd.y, gd.z, gd.angle, gd.width
    );

    // 1) 禁用碰撞
    disableAllCollisions();

    // 2) 暂停环境建模
    stopEnvironmentUpdate();

    // 3) 机械臂移动到抓取点(带上 angle)
    if(!moveArmToPose(gd.x, gd.y, gd.z, gd.angle))
    {
        RCLCPP_ERROR(this->get_logger(), "[onPickClicked] moveArmToPose failed => abort!");
        return;
    }

    // ----------- 新增：沿着X轴再向前 6 cm -----------
    if(!moveArmRelativeOffset(0.06, 0.0, 0.0))
    {
        RCLCPP_ERROR(this->get_logger(), "[onPickClicked] moveArmRelativeOffset failed => abort!");
        return;
    }

    // 4) 爪子先张开到识别到的宽度
    moveGripper(gd.width);

    // 5) 再收紧一点（原逻辑加 0.06）
    float narrower = gd.width + 0.06f;
    RCLCPP_INFO(this->get_logger(), "Narrower=%.3f => now calling moveGripper(narrower).", narrower);
    moveGripper(narrower);

    RCLCPP_INFO(this->get_logger(), "[onPickClicked] Done.");
}

//---------------------------------------
// [New] PickWA (忽略 angle)
//---------------------------------------
void QtNode::onPickWAClicked()
{
    auto item = object_list_->currentItem();
    if(!item)
    {
        RCLCPP_WARN(this->get_logger(), "No item selected in list => abort pickWA!");
        return;
    }
    std::string label = item->text().toStdString();

    GraspData gd;
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        if(!label2grasp_.count(label))
        {
            RCLCPP_WARN(this->get_logger(), "No grasp data for label=%s => abort pickWA!", label.c_str());
            return;
        }
        gd = label2grasp_[label];
    }

    // 忽略 gd.angle，直接固定 0.0
    float fixed_angle = 0.0f;

    RCLCPP_INFO(this->get_logger(),
        "[onPickWAClicked] ignoring angle=%.2f => use fixed=%.2f => (x=%.3f,y=%.3f,z=%.3f,width=%.3f)",
        gd.angle, fixed_angle, gd.x, gd.y, gd.z, gd.width
    );

    // 1) 禁用碰撞
    disableAllCollisions();

    // 2) 暂停环境建模
    stopEnvironmentUpdate();

    // 3) 机械臂移动到抓取点(使用 fixed_angle)
    if(!moveArmToPose(gd.x, gd.y, gd.z, fixed_angle))
    {
        RCLCPP_ERROR(this->get_logger(), "[onPickWAClicked] moveArmToPose failed => abort!");
        return;
    }



    // 4) 爪子张开
    moveGripper(gd.width);

    // 5) 再收紧
    float narrower = gd.width + 0.06f;
    RCLCPP_INFO(this->get_logger(), "PickWA => narrower=%.3f => moveGripper(narrower).", narrower);
    moveGripper(narrower);

    RCLCPP_INFO(this->get_logger(), "[onPickWAClicked] Done.");
}

//---------------------------------------
// Deliver
//---------------------------------------
void QtNode::onDeliverClicked()
{
    // ---------------- 新增：joint2 在当前角度基础上 +15度 ----------------
    if(!arm_)
    {
        RCLCPP_ERROR(this->get_logger(), "[onDeliverClicked] arm_ not initialized => abort!");
        return;
    }

    // 1) 获取当前关节角
    std::vector<double> current_joints = arm_->getCurrentJointValues();
    //   假定第2个关节为 joint2
    if(current_joints.size() < 2)
    {
        RCLCPP_ERROR(this->get_logger(), "[onDeliverClicked] Not enough joints => abort!");
        return;
    }
    //   在当前角度基础上 +15度（转为弧度）
    current_joints[1] += (15.0 * M_PI / 180.0);

    //   Move
    if(!moveArmToJoints(current_joints))
    {
        RCLCPP_ERROR(this->get_logger(), "[onDeliverClicked] Move to current_joints +15deg failed => abort!");
        return;
    }

    // 2) 再执行原先设定的递送姿态
    double j1 = -50.0*M_PI/180.0;
    double j2 = 0.0;  // 不变时, 就是原来的 deliver 角度
    double j3 = -25.0*M_PI/180.0;
    double j4 = 0.0;
    double j5 = 25.0*M_PI/180.0;
    double j6 = 0.0;
    std::vector<double> joints{j1, j2, j3, j4, j5, j6};

    if(!moveArmToJoints(joints))
    {
        RCLCPP_ERROR(this->get_logger(), "[onDeliverClicked] moveArmToJoints failed");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "[onDeliverClicked] Done.");
}

//---------------------------------------
// Clear
//---------------------------------------
void QtNode::onClearClicked()
{
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        label2grasp_.clear();
    }
    object_list_->clear();
    RCLCPP_INFO(this->get_logger(), "All items cleared.");
}

//---------------------------------------
// Release
//---------------------------------------
void QtNode::onReleaseClicked()
{
    auto item = object_list_->currentItem();
    if(!item)
    {
        RCLCPP_WARN(this->get_logger(),
            "No item selected in list for release => abort!");
        return;
    }

    std::string label = item->text().toStdString();
    RCLCPP_INFO(this->get_logger(),
        "[onReleaseClicked] label=%s", label.c_str());

    GraspData gd;
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        if(!label2grasp_.count(label))
        {
            RCLCPP_WARN(this->get_logger(),
                "No grasp data for label=%s => abort release!", label.c_str());
            return;
        }
        gd = label2grasp_[label];
    }

    float release_w = gd.width - 0.03f;
    moveGripper(release_w);

    // 恢复碰撞
    enableAllCollisionsBack();
    // 恢复环境建模
    resumeEnvironmentUpdate();

    RCLCPP_INFO(this->get_logger(), "[onReleaseClicked] done.");
}

//---------------------------------------
// onChooseClicked
//---------------------------------------
void QtNode::onChooseClicked()
{
    auto item = object_list_->currentItem();
    if(!item)
    {
        RCLCPP_WARN(this->get_logger(), "No item selected => abort choose!");
        return;
    }
    std::string label = item->text().toStdString();

    GraspData gd;
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        if(!label2grasp_.count(label))
        {
            RCLCPP_WARN(this->get_logger(),
                "[onChooseClicked] no grasp data => abort!");
            return;
        }
        gd = label2grasp_[label];
    }

    RCLCPP_INFO(this->get_logger(),
        "[onChooseClicked] => Move arm so object label=%s is centered. angle=%.2f",
        label.c_str(), gd.angle
    );

    // 简单移动到 (x,y,z+0.05)
    if(!moveArmToPose(gd.x, gd.y, gd.z + 0.05f, gd.angle))
    {
        RCLCPP_ERROR(this->get_logger(),
            "[onChooseClicked] moveArmToPose failed => abort!");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "[onChooseClicked] done!");
}

//---------------------------------------
// onEnvClicked
//---------------------------------------
void QtNode::onEnvClicked()
{
    RCLCPP_INFO(this->get_logger(), "[onEnvClicked] => run scanning program...");

    // 示例：运行外部可执行
    std::string cmd = "ros2 run sgr532_moveit_demo sgr532_pick_place";
    int ret = system(cmd.c_str());
    if(ret == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to run env scanning program!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Env scanning program finished or running...");
    }
}

//---------------------------------------
// 回调: GraspPoints
//---------------------------------------
void QtNode::graspCallback(const my_bridge_pkg::msg::GraspPoint::SharedPtr msg)
{
    if(msg->object_label.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Empty GraspPoint => do nothing");
        return;
    }
    std::lock_guard<std::mutex> lk(data_mutex_);
    for(size_t i=0; i<msg->object_label.size(); i++)
    {
        const std::string &lbl = msg->object_label[i];
        if(lbl.empty()) continue;

        if(!label2grasp_.count(lbl))
        {
            label2grasp_[lbl] = {
                msg->x[i], msg->y[i], msg->z[i],
                msg->angle[i], msg->width[i],
                msg->score[i], msg->confidence[i]
            };
        }
        else
        {
            // 如果新的 score 更高就更新
            auto &old_g = label2grasp_[lbl];
            if(msg->score[i] > old_g.score)
            {
                old_g.x = msg->x[i];
                old_g.y = msg->y[i];
                old_g.z = msg->z[i];
                old_g.angle = msg->angle[i];
                old_g.width = msg->width[i];
                old_g.score = msg->score[i];
                old_g.confidence = msg->confidence[i];
            }
        }
    }
}

//---------------------------------------
// 回调: 原始图像
//---------------------------------------
void QtNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch(...)
    {
        return;
    }
    cv::Mat frame = cv_ptr->image;
    if(frame.empty()) return;

    QImage qimg(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_BGR888);
    QPixmap pixmap = QPixmap::fromImage(qimg.copy()).scaled(
        rgb_label_->width(),
        rgb_label_->height(),
        Qt::KeepAspectRatio
    );
    rgb_label_->setPixmap(pixmap);
}

//---------------------------------------
// 回调: 标注图像
//---------------------------------------
void QtNode::annotatedImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch(...)
    {
        return;
    }
    cv::Mat ann = cv_ptr->image;
    if(ann.empty()) return;

    QImage qimg(ann.data, ann.cols, ann.rows, ann.step, QImage::Format_BGR888);
    QPixmap pixmap = QPixmap::fromImage(qimg.copy()).scaled(
        bottom_label_->width(),
        bottom_label_->height(),
        Qt::KeepAspectRatio
    );
    bottom_label_->setPixmap(pixmap);
}

//---------------------------------------
// updateUI
//---------------------------------------
void QtNode::updateUI()
{
    // 记录选中的
    std::string old_selected;
    if(auto current = object_list_->currentItem())
    {
        old_selected = current->text().toStdString();
    }

    // 拷贝
    std::unordered_map<std::string, GraspData> local_map;
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        local_map = label2grasp_;
    }

    // 重建列表：只显示score>0
    object_list_->clear();
    for(const auto &kv : local_map)
    {
        if(kv.second.score > 0.0f)
        {
            object_list_->addItem(QString::fromStdString(kv.first));
        }
    }

    // 恢复选中
    if(!old_selected.empty() && local_map.count(old_selected))
    {
        int countRow = object_list_->count();
        for(int i=0; i<countRow; i++)
        {
            if(object_list_->item(i)->text().toStdString() == old_selected)
            {
                object_list_->setCurrentRow(i);
                break;
            }
        }
    }
    else
    {
        if(object_list_->count() > 0)
            object_list_->setCurrentRow(0);
    }
}

//---------------------------------------
// moveArmToPose
//---------------------------------------
bool QtNode::moveArmToPose(float x, float y, float z, float yaw_rad)
{
    if(!arm_)
    {
        RCLCPP_ERROR(this->get_logger(), "[moveArmToPose] arm_ not initialized!");
        return false;
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    // 这里注意：如果想绕Z轴旋转，需要改成 setRPY(0, 0, yaw_rad)
    // 目前是 setRPY(yaw_rad, 0, 0) => 绕X轴旋转
    tf2::Quaternion q;
    q.setRPY(yaw_rad, 0.0, 0.0);
    target_pose.orientation = tf2::toMsg(q);

    arm_->setStartStateToCurrentState();
    arm_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success)
    {
        auto code = arm_->execute(plan);
        return (code == moveit::core::MoveItErrorCode::SUCCESS);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),
            "[moveArmToPose] => plan failed => (%.3f,%.3f,%.3f,yaw=%.2f) not reachable?",
            x,y,z,yaw_rad
        );
        return false;
    }
}

//---------------------------------------
// moveArmToJoints
//---------------------------------------
bool QtNode::moveArmToJoints(const std::vector<double> &joint_values)
{
    if(!arm_)
    {
        RCLCPP_ERROR(this->get_logger(), "[moveArmToJoints] arm_ not initialized!");
        return false;
    }

    auto jmg = arm_->getCurrentState()->getJointModelGroup("arm");
    if(!jmg)
    {
        RCLCPP_ERROR(this->get_logger(), "[moveArmToJoints] no jmg named 'arm'!");
        return false;
    }
    auto jn = jmg->getVariableNames();
    if(jn.size() != joint_values.size())
    {
        RCLCPP_ERROR(this->get_logger(),
            "[moveArmToJoints] mismatch => jn.size()=%zu, input.size()=%zu",
            jn.size(), joint_values.size());
        return false;
    }

    for(size_t i=0; i<jn.size(); i++)
    {
        arm_->setJointValueTarget(jn[i], joint_values[i]);
    }
    arm_->setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success)
    {
        auto code = arm_->execute(plan);
        return (code == moveit::core::MoveItErrorCode::SUCCESS);
    }
    return false;
}

//---------------------------------------
// moveGripper
//---------------------------------------
bool QtNode::moveGripper(double width)
{
    try
    {
        if(!gripper_)
        {
            RCLCPP_ERROR(this->get_logger(),
                "[moveGripper] gripper_ not initialized!");
            return false;
        }

        auto joint_names = gripper_->getJointNames();
        if (joint_names.size() < 2)
        {
            RCLCPP_ERROR(this->get_logger(),
                "[moveGripper] Gripper joint config error => only %zu joints?",
                joint_names.size());
            return false;
        }

        // 设定：两爪对称，width = (爪左 - 爪右)，故每个爪子是 -width/2
        double pos = -width / 2.0;
        std::vector<double> joint_positions{pos, pos};

        RCLCPP_INFO(this->get_logger(),
            "[moveGripper] => width=%.4f => (left=%.4f, right=%.4f)",
            width, joint_positions[0], joint_positions[1]);

        gripper_->setJointValueTarget(joint_names[0], joint_positions[0]);
        gripper_->setJointValueTarget(joint_names[1], joint_positions[1]);
        gripper_->setPlanningTime(1.0);

        auto error_code = gripper_->move();
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "[moveGripper] success => width=%.3f", width);
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                "[moveGripper] failed => code=%d => width=%.3f",
                error_code.val, width);
            return false;
        }
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "[moveGripper] Exception: %s", e.what());
        return false;
    }
}

//---------------------------------------
// moveArmRelativeOffset
//   在当前末端姿态的基础上，沿 X/Y/Z 做一个相对位移(单位:米)
//---------------------------------------
bool QtNode::moveArmRelativeOffset(double dx, double dy, double dz)
{
    if(!arm_)
    {
        RCLCPP_ERROR(this->get_logger(), "[moveArmRelativeOffset] arm_ not initialized!");
        return false;
    }
    // 读取当前Pose
    geometry_msgs::msg::Pose current_pose = arm_->getCurrentPose().pose;
    // 改变 position
    current_pose.position.x += dx;
    current_pose.position.y += dy;
    current_pose.position.z += dz;

    arm_->setStartStateToCurrentState();
    arm_->setPoseTarget(current_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success)
    {
        auto code = arm_->execute(plan);
        return (code == moveit::core::MoveItErrorCode::SUCCESS);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),
            "[moveArmRelativeOffset] => plan failed => offset(%.3f, %.3f, %.3f) not reachable?",
            dx, dy, dz
        );
        return false;
    }
}

//---------------------------------------
// disableAllCollisions
//---------------------------------------
void QtNode::disableAllCollisions()
{
    if(!planning_scene_monitor_)
    {
        RCLCPP_WARN(this->get_logger(),
            "No planning_scene_monitor_ => cannot disable collisions => return!");
        return;
    }

    planning_scene_monitor::LockedPlanningSceneRW locked_scene(planning_scene_monitor_);
    auto &acm = locked_scene->getAllowedCollisionMatrixNonConst();

    auto robot_links = locked_scene->getRobotModel()->getLinkModelNames();
    auto world = locked_scene->getWorld();
    auto object_ids = world->getObjectIds();

    for(const auto &obj_id : object_ids)
    {
        for(const auto &rlink : robot_links)
        {
            acm.setEntry(rlink, obj_id, true);
        }
    }

    planning_scene_monitor_->triggerSceneUpdateEvent(
        planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_SCENE);

    moveit_msgs::msg::PlanningScene scene_msg;
    locked_scene->getPlanningSceneMsg(scene_msg);
    scene_msg.is_diff = true;

    if(!apply_planning_scene_client_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(this->get_logger(),
            "disableAllCollisions: /apply_planning_scene not available!");
        return;
    }
    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    req->scene = scene_msg;

    auto future = apply_planning_scene_client_->async_send_request(req);
    if(future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    {
        auto resp = future.get();
        if(resp->success)
        {
            RCLCPP_WARN(this->get_logger(),
                "Collisions DISABLED for robot vs ALL environment!");
        }
    }
}

//---------------------------------------
// enableAllCollisionsBack
//---------------------------------------
void QtNode::enableAllCollisionsBack()
{
    if(!planning_scene_monitor_)
    {
        RCLCPP_WARN(this->get_logger(),
            "No planning_scene_monitor_ => cannot re-enable collisions => return!");
        return;
    }

    planning_scene_monitor::LockedPlanningSceneRW locked_scene(planning_scene_monitor_);
    auto &acm = locked_scene->getAllowedCollisionMatrixNonConst();

    // 恢复默认
    acm = collision_detection::AllowedCollisionMatrix();

    planning_scene_monitor_->triggerSceneUpdateEvent(
        planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType::UPDATE_SCENE);

    moveit_msgs::msg::PlanningScene scene_msg;
    locked_scene->getPlanningSceneMsg(scene_msg);
    scene_msg.is_diff = true;

    if(!apply_planning_scene_client_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(this->get_logger(),
            "enableAllCollisionsBack: /apply_planning_scene not available!");
        return;
    }
    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    req->scene = scene_msg;

    auto future = apply_planning_scene_client_->async_send_request(req);
    if(future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
    {
        auto resp = future.get();
        if(resp->success)
        {
            RCLCPP_INFO(this->get_logger(),
                "Collisions RE-ENABLED to default!");
        }
    }
}

//---------------------------------------
// stopEnvironmentUpdate
//---------------------------------------
void QtNode::stopEnvironmentUpdate()
{
    if(!planning_scene_monitor_)
    {
        RCLCPP_WARN(this->get_logger(),
            "stopEnvironmentUpdate: no planning_scene_monitor_ => do nothing!");
        return;
    }

    // 停止对 collision_object / planning_scene_world 的监控
    planning_scene_monitor_->stopWorldGeometryMonitor();
}

//---------------------------------------
// resumeEnvironmentUpdate
//---------------------------------------
void QtNode::resumeEnvironmentUpdate()
{
    if(!planning_scene_monitor_)
    {
        RCLCPP_WARN(this->get_logger(),
            "resumeEnvironmentUpdate: no planning_scene_monitor_ => do nothing!");
        return;
    }

    // 恢复对 collision_object / planning_scene_world 的监控
    planning_scene_monitor_->startWorldGeometryMonitor();
}