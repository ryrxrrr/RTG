# RTG
A semantic grasping pipeline built on ROS 2, leveraging YOLOv8x for object-aware segmentation and a U-Net-based CNN with MHSA and CBAM for grasp prediction. Trained with augmented Jacquard v2 data and executed via MoveIt 2, it enables robust manipulation in unstructured environments.

🌟Key Innovations
1. Semantic segmentation using YOLOv8x to guide object-aware grasping;
2. Enhanced CNN grasp predictor based on U-Net with:
   MHSA (Multi-Head Self-Attention) for long-range spatial context;
   CBAM attention for adaptive feature refinement;
   Data augmentation on Jacquard v2: mild 3D rotation, noise, and exposure distortion for 
      improved real-world robustness;
3. Grasp predictions filtered by a Kalman filter, improving stability and reducing execution failures due to noisy depth inputs or prediction jitter;
4.Migrated original ROS 1 robot driver to ROS 2, with full MoveIt 2 integration for planning and execution;Trajectory planning and collision-aware motion control via MoveIt 2;
5.Custom-built Qt-based Linux GUI for grasp control and visualization.

🌟Since I could not find any published paper or existing robotic arm system that performs "arbitrary object grasping in natural environments", there is currently no baseline for direct comparison. As a result, I have not yet conducted quantitative benchmarking to measure the performance improvement. However, I plan to define and implement a proper evaluation method soon.
🌟This is my undergraduate final year project, and I acknowledge that some parts may be suboptimal or incomplete due to time and experience constraints. If you find any issues or have suggestions, I would sincerely appreciate your feedback. Thank you soooooo much！！！！！ and I apologis for any inconvenience


📦 Package Descriptions
1.ggcnn
Used for grasp point prediction. The model is trained on the Jacquard v2 dataset, and outputs grasp poses with pixel-level precision.
2.moveit_config
Configuration package for MoveIt 2, including robot kinematics, planning pipelines, controller setup, and RViz visualization.
3.my_bridge_pkg
A custom bridge node that transfers detected object grasp points from the vision module to the interaction interface.
4.object_interface
A Qt-based GUI interface for user interaction. Allows users to visualize detected objects, control grasp actions, and view system status.
5.realsense-ros
Official ROS 2 wrapper for the Intel RealSense D405 depth camera. Provides RGB-D image streams and camera info.
6.sagittarius_descriptions
Contains URDF and mesh files for the Sagittarius robotic arm. Used in simulation and visualization (RViz, MoveIt).
7.sdk_sagittarius_arm
The hardware SDK interface responsible for establishing communication between the PC and the Sagittarius arm. Includes motion APIs, message formats, and launch scripts.
8.sgr532_moveit_demo
A demonstration package for MoveIt-based planning and execution of predefined motions. Includes trajectory recording and playback for repeatable tasks.
9.yolov8_infer
Runs YOLOv8 for real-time object detection and segmentation. Generates semantic masks that assist in category-aware grasping.

🌟 How to Use This Project
  🚀1.environment: ubuntu 22.04LTS, Ros2 humble. moveit v2.5.6 (could download from:
      https://drive.google.com/file/d/1i5Cp6JulgT3efNkkyQpPAY9GBIjOxgwq/view?usp=sharing)
    2.creat a folder to put "src" file, for example("sagittarius_ws-main“）
      put "ws_moveit2"( the fild download from google drive) and "sagittarius_ws-main“ parellely.
  
  🚀 You could also download full project from google drive link below: 
       https://drive.google.com/file/d/1MuyXaOqRw7RYu0Ut04jqai4sWxkhOfQ6/view?usp=drive_link

# Sagittarius Arm – Full Bring‑Up Procedure (ROS 2 Humble)
  > **Note**  The very first `colcon build` may take several minutes.

## 1  Prepare *six* terminals
Run the same initialization commands in **each** of the six terminals:

cd ~/sagittarius_ws-main
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash
colcon build             # first build can be slow
source install/setup.bash


## 2  Terminal 1 – Start the robot driver
Launch the communication node between the Sagittarius arm and the PC:

ros2 launch sdk_sagittarius_arm sdk_sagittarius_arm.launch.py

## 3  Terminal 2 – Start the Intel RealSense D405 (depth + TF)

ros2 launch realsense2_camera rs_align_depth_launch.py \
  camera_name:=camera \
  publish_tf:=true \
  base_frame_id:=camera_link \
  color_frame_id:=camera_color_frame \
  color_optical_frame_id:=camera_color_optical_frame \
  depth_frame_id:=camera_depth_frame \
  depth_optical_frame_id:=camera_depth_optical_frame \
  filters:=pointcloud \
  pointcloud.enable:=true \
  pointcloud.stream_filter:=color \
  pointcloud.stream_index_filter:=0 \
  use_system_timestamp:=true \
  enable_sync:=true \
  depth_fps:=30 \
  color_fps:=30 \
  use_sim_time:=false

## 4  Terminal 3 – Launch MoveIt 2 demo

ros2 launch moveit_config demo.launch.py

## 5  Terminal 4 – Activate the virtual env & start YOLOv8 + GG‑CNN

cd ~/sagittarius_ws-main
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
export PYTHONPATH=/home/royin/sagittarius_ws-main/src:$PYTHONPATH
ros2 run yolov8_infer yolo_infer

## 6  Terminal 5 – Bridge perception results to the UI (with Kalman filter)

ros2 launch my_bridge_pkg bridge_launch.py

## 7  Terminal 6 – Start the object‑interaction UI

ros2 launch object_interface ui.launch.py

## Optional – Fixed trajectory pick‑and‑place demo
With the MoveIt demo already running, execute:

ros2 run sgr532_moveit_demo sgr532_pick_place



