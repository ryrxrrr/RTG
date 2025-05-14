# RTG
A semantic grasping pipeline built on ROS 2, leveraging YOLOv8x for object-aware segmentation and a U-Net-based CNN with MHSA and CBAM for grasp prediction. Trained with augmented Jacquard v2 data and executed via MoveIt 2, it enables robust manipulation in unstructured environments.

🌟Key Innovations
Semantic segmentation using YOLOv8x to guide object-aware grasping;

Enhanced CNN grasp predictor based on U-Net with:

MHSA (Multi-Head Self-Attention) for long-range spatial context;

CBAM attention for adaptive feature refinement;

Data augmentation on Jacquard v2: mild 3D rotation, noise, and exposure distortion for improved real-world robustness;

Grasp predictions filtered by a Kalman filter, improving stability and reducing execution failures due to noisy depth inputs or prediction jitter;

Migrated original ROS 1 robot driver to ROS 2, with full MoveIt 2 integration for planning and execution;

Trajectory planning and collision-aware motion control via MoveIt 2;

Custom-built Qt-based Linux GUI for grasp control and visualization.

🌟Since I could not find any published paper or existing robotic arm system that performs "arbitrary object grasping in natural environments", there is currently no baseline for direct comparison. As a result, I have not yet conducted quantitative benchmarking to measure the performance improvement. However, I plan to define and implement a proper evaluation method soon.

🌟This is my undergraduate final year project, and I acknowledge that some parts may be suboptimal or incomplete due to time and experience constraints. If you find any issues or have suggestions, I would sincerely appreciate your feedback. Thank you soooooo much！！！！！
