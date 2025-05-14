#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

# ROS2 消息类型
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# YOLOv8
from ultralytics import YOLO
import torch

# 其他Python库
import numpy as np
import cv2
import sys
import os

# 同时订阅多个话题并做时间同步
from message_filters import Subscriber, ApproximateTimeSynchronizer

# ========== 引入自定义消息 ==========
from object_interface.msg import DetectionInfo

# ========== GG-CNN 部分 ========== 
# 假设你的 ggcnn 仓库在 ~/sagittarius_ws-main/src/ggcnn
ggcnn_path = os.path.join(os.path.expanduser('~'), 'sagittarius_ws-main', 'src', 'ggcnn')
sys.path.append(ggcnn_path)
from models.ggcnn import GGCNN  # 若该文件中定义了 GGCNN 类


class KalmanFilterGrasp:
    """
    使用线性卡尔曼滤波器来平滑跟踪:
      State = [x, y, z, angle, width, vx, vy, vz, v_angle, v_width]^T
    假设:
      - 观测(测量)只提供 [x, y, z, angle, width] (score仅判断是否有效，不进入KF向量)
      - 运动模型: 位置 = 位置 + 速度 * dt (匀速模型)
      - dt 在此示例中统一设为1，可根据帧率做更精确的处理
    """
    def __init__(self):
        self.dim_state = 10  # x, y, z, angle, width + (vx,vy,vz,vangle,vwidth)
        self.dim_meas = 5
        self.initialized = False

        # 状态向量
        self.x = np.zeros((self.dim_state, 1), dtype=np.float32)
        # 状态协方差矩阵
        self.P = np.eye(self.dim_state, dtype=np.float32) * 1e3

        # 状态转移矩阵F (dt=1)
        self.F = np.eye(self.dim_state, dtype=np.float32)
        # 位置 = 位置 + 速度
        self.F[0, 5] = 1.0  # x += vx
        self.F[1, 6] = 1.0  # y += vy
        self.F[2, 7] = 1.0  # z += vz
        self.F[3, 8] = 1.0  # angle += v_angle
        self.F[4, 9] = 1.0  # width += v_width

        # 观测矩阵H
        # z = [x, y, z, angle, width] = H * x
        self.H = np.zeros((self.dim_meas, self.dim_state), dtype=np.float32)
        self.H[0, 0] = 1.0  # x
        self.H[1, 1] = 1.0  # y
        self.H[2, 2] = 1.0  # z
        self.H[3, 3] = 1.0  # angle
        self.H[4, 4] = 1.0  # width

        # 过程噪声协方差 Q (可根据经验调整)
        self.Q = np.diag([
            1e-4, 1e-4, 1e-4, 1e-5, 1e-5,  # x, y, z, angle, width
            1e-3, 1e-3, 1e-3, 1e-4, 1e-4   # vx, vy, vz, v_angle, v_width
        ]).astype(np.float32)

        # 测量噪声协方差 R (可根据相机测量、抓取网络精度调整)
        self.R = np.diag([
            1e-3, 1e-3, 1e-3, 1e-4, 1e-4
        ]).astype(np.float32)

    def init_state(self, meas):
        """
        用测量值初始化卡尔曼滤波器
        meas: [x, y, z, angle, width]
        """
        self.x[:5, 0] = meas[:5]
        self.P = np.eye(self.dim_state, dtype=np.float32) * 1.0
        self.initialized = True

    def predict(self):
        """
        KF预测
          x = F*x
          P = F*P*F^T + Q
        """
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        # 对角度做wrap
        self.x[3, 0] = self._wrap_angle(self.x[3, 0])

    def update(self, meas):
        """
        KF更新(线性)
        meas: [x, y, z, angle, width]
        """
        z = meas.reshape(-1,1)  # (5,1)

        # 计算卡尔曼增益
        S = self.H @ self.P @ self.H.T + self.R  # (5,5)
        K = self.P @ self.H.T @ np.linalg.inv(S) # (10,5)

        # y = z - Hx
        y = z - (self.H @ self.x)
        # 对测量角度做wrap
        y[3,0] = self._wrap_angle(y[3,0])

        # 更新状态
        self.x = self.x + K @ y

        # 更新协方差
        I = np.eye(self.dim_state, dtype=np.float32)
        self.P = (I - K @ self.H) @ self.P

        # 再wrap一次角度
        self.x[3, 0] = self._wrap_angle(self.x[3, 0])

    def get_state(self):
        """
        返回平滑后的 [x, y, z, angle, width]
        """
        return self.x[0,0], self.x[1,0], self.x[2,0], self.x[3,0], self.x[4,0]

    @staticmethod
    def _wrap_angle(angle):
        """让角度落在 [-pi, pi] 范围内"""
        return (angle + np.pi) % (2.0 * np.pi) - np.pi


class YoloInfer3DNode(Node):
    def __init__(self):
        """
        本节点命名 'yolo_infer_3d_node'，同步订阅:
          1) 彩色图:   /camera/camera/color/image_rect_raw
          2) 对齐深度: /camera/camera/aligned_depth_to_color/image_raw
          3) 相机内参: /camera/camera/color/camera_info

        在回调中:
          - YOLO 检测 (彩色图)
          - ROI 深度 -> GG-CNN 推理 -> 抓取姿态
          - 若抓取评分 >= 阈值，则进行卡尔曼滤波更新; 否则只预测不更新
          - 投影到3D并可视化(包括KF平滑结果)
          - **将最终结果以 DetectionInfo 消息方式发布到 /detection_info**.
        """
        super().__init__('yolo_infer_3d_node')
        self.get_logger().info("Initializing YoloInfer3DNode...")

        # A) CVBridge: ROS图像 <-> OpenCV
        self.bridge = CvBridge()

        # B) 声明3个订阅者(利用 message_filters 做时间同步)
        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_rect_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.info_sub  = Subscriber(self, CameraInfo, '/camera/camera/color/camera_info')

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        # C) 加载 YOLOv8
        self.model = YOLO('yolov8x.pt')
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info("Using GPU for YOLO inference.")
        else:
            self.get_logger().info("GPU not available, using CPU for YOLO.")

        # D) 加载 GG-CNN
        self.ggcnn_model = GGCNN()
        weights_file = os.path.join(ggcnn_path, 'ggcnn_weights_cornell', 'ggcnn_epoch_23_cornell_statedict.pt')
        self.ggcnn_model.load_state_dict(torch.load(weights_file, map_location='cpu'))
        self.ggcnn_model.eval()
        if torch.cuda.is_available():
            self.ggcnn_model.to('cuda')
            self.get_logger().info("Using GPU for GG-CNN inference.")
        else:
            self.get_logger().info("GPU not available, using CPU for GG-CNN.")

        # E) 深度缩放比例(根据实际相机)
        self.depth_scale = 0.001

        # F) 卡尔曼滤波器
        self.kf = KalmanFilterGrasp()

        # G) GG-CNN输入图像尺寸
        self.ggcnn_input_size = 300

        # H) 创建一个Publisher，发布 DetectionInfo 消息到 /detection_info
        self.detection_pub = self.create_publisher(
            DetectionInfo, 
            '/detection_info', 
            10
        )

        self.get_logger().info("YoloInfer3DNode initialized successfully.")

    def sync_callback(self, color_msg, depth_msg, info_msg):
        """
        当彩色图、对齐深度图、相机内参三者时间近似对齐后，被调用。
        """
        # 1) ROS图像 -> OpenCV
        color_frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # 2) 从CameraInfo取出内参
        K = info_msg.k  # 3x3
        fx, fy = K[0], K[4]
        cx, cy = K[2], K[5]

        # 3) YOLOv8 推理 (仅演示处理第一个检测目标)
        results = self.model.predict(color_frame, conf=0.2)
        if len(results) == 0:
            # 没有检测到物体
            if self.kf.initialized:
                self.kf.predict()
            cv2.imshow("YOLOv8 + GG-CNN + Kalman", color_frame)
            cv2.waitKey(1)
            return

        result = results[0]
        boxes = result.boxes  # list of bboxes
        names = result.names  # 类别名映射

        annotated = color_frame.copy()

        # 只演示抓取第一个box
        box = boxes[0]
        b = box.xyxy[0].cpu().numpy().astype(int)  # [x1, y1, x2, y2]
        x1, y1, x2, y2 = b
        cls_id = int(box.cls[0])
        conf   = float(box.conf[0])
        label  = names[cls_id]

        cv2.rectangle(annotated, (x1,y1), (x2,y2), (0,255,0), 2)
        cv2.putText(annotated, f"{label} {conf:.2f}", 
                    (x1, max(y1-10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # 4) 目标框中心
        cx_box = (x1 + x2) // 2
        cy_box = (y1 + y2) // 2

        # 获取深度
        if (0 <= cy_box < depth_frame.shape[0]) and (0 <= cx_box < depth_frame.shape[1]):
            d_raw = depth_frame[cy_box, cx_box]
        else:
            d_raw = 0

        if d_raw == 0:
            if self.kf.initialized:
                self.kf.predict()
            cv2.imshow("YOLOv8 + GG-CNN + Kalman", annotated)
            cv2.waitKey(1)
            return

        dist_m = d_raw * self.depth_scale
        # 投影到相机坐标
        X = (cx_box - cx) * dist_m / fx
        Y = (cy_box - cy) * dist_m / fy
        Z = dist_m

        # 在图像上显示中心距离
        cv2.putText(annotated, f"Dist={dist_m:.3f}m", (x1,y1+15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

        # 5) GG-CNN 抓取推理
        roi_depth = depth_frame[y1:y2, x1:x2].copy()
        if roi_depth.size == 0:
            if self.kf.initialized:
                self.kf.predict()
            cv2.imshow("YOLOv8 + GG-CNN + Kalman", annotated)
            cv2.waitKey(1)
            return

        roi_depth_m = roi_depth.astype(np.float32) * self.depth_scale

        # resize到模型要求
        resized_roi = cv2.resize(roi_depth_m, (self.ggcnn_input_size, self.ggcnn_input_size),
                                 interpolation=cv2.INTER_NEAREST)
        roi_tensor = torch.from_numpy(resized_roi).unsqueeze(0).unsqueeze(0)
        if torch.cuda.is_available():
            roi_tensor = roi_tensor.cuda()

        with torch.no_grad():
            pos_out, cos_out, sin_out, width_out = self.ggcnn_model(roi_tensor)

        pos_map   = pos_out[0,0].cpu().numpy()
        cos_map   = cos_out[0,0].cpu().numpy()
        sin_map   = sin_out[0,0].cpu().numpy()
        width_map = width_out[0,0].cpu().numpy()

        max_q_idx = np.unravel_index(np.argmax(pos_map), pos_map.shape)
        grasp_q = pos_map[max_q_idx]
        grasp_angle = np.arctan2(sin_map[max_q_idx], cos_map[max_q_idx])
        grasp_width = width_map[max_q_idx]

        if grasp_q < 0.1:
            # 评分不足
            if self.kf.initialized:
                self.kf.predict()
            cv2.imshow("YOLOv8 + GG-CNN + Kalman", annotated)
            cv2.waitKey(1)
            return

        # 映射回原图像坐标
        (gy, gx) = max_q_idx
        h_roi, w_roi = roi_depth_m.shape
        scale_y = h_roi / float(self.ggcnn_input_size)
        scale_x = w_roi / float(self.ggcnn_input_size)
        real_y = int(gy * scale_y)
        real_x = int(gx * scale_x)
        center_x = x1 + real_x
        center_y = y1 + real_y

        # 再取一次深度 (可选)
        d_pt = 0
        if (0 <= center_y < depth_frame.shape[0]) and (0 <= center_x < depth_frame.shape[1]):
            d_pt = depth_frame[center_y, center_x]
        if d_pt == 0:
            d_pt = d_raw
        dist_pt_m = d_pt * self.depth_scale

        Gx = (center_x - cx) * dist_pt_m / fx
        Gy = (center_y - cy) * dist_pt_m / fy
        Gz = dist_pt_m

        # 6) 卡尔曼滤波
        if self.kf.initialized:
            self.kf.predict()

        meas = np.array([Gx, Gy, Gz, grasp_angle, grasp_width], dtype=np.float32)
        if not self.kf.initialized:
            self.kf.init_state(meas)
        else:
            self.kf.update(meas)

        kf_x, kf_y, kf_z, kf_ang, kf_w = self.kf.get_state()

        # (G) 可视化
        cv2.drawMarker(annotated, (center_x, center_y), (0,0,255), 
                       markerType=cv2.MARKER_TILTED_CROSS, markerSize=20, thickness=2)
        cv2.putText(annotated, f"GQ={grasp_q:.2f}, Ang={np.degrees(grasp_angle):.1f}",
                    (center_x, center_y - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

        self.get_logger().info(
            f"[KF] x={kf_x:.3f}, y={kf_y:.3f}, z={kf_z:.3f}, "
            f"ang={np.degrees(kf_ang):.1f} deg, w={kf_w:.3f}, q={grasp_q:.2f}"
        )

        # 在图像上标出KF结果
        if kf_z > 0:
            u_kf = int((kf_x * fx / kf_z) + cx)
            v_kf = int((kf_y * fy / kf_z) + cy)
            if 0 <= v_kf < annotated.shape[0] and 0 <= u_kf < annotated.shape[1]:
                cv2.drawMarker(annotated, (u_kf, v_kf), (255,0,0),
                               markerType=cv2.MARKER_TILTED_CROSS, markerSize=25, thickness=2)
                cv2.putText(annotated, "KF", (u_kf+5, v_kf+5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

        # 7) 发布 DetectionInfo 消息
        #   object_name = YOLO识别到的类别
        #   其余从 KF 平滑后获取
        det_msg = DetectionInfo()
        det_msg.object_name = label
        det_msg.x      = float(kf_x)
        det_msg.y      = float(kf_y)
        det_msg.z      = float(kf_z)
        det_msg.angle  = float(kf_ang)
        det_msg.width  = float(kf_w)
        det_msg.score  = float(grasp_q)

        self.detection_pub.publish(det_msg)
        self.get_logger().info(f"Published detection: {label}, score={grasp_q:.2f}")

        # 8) 显示最终画面
        cv2.imshow("YOLOv8 + GG-CNN + Kalman", annotated)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloInfer3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
