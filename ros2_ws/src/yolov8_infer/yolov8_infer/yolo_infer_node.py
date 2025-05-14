#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

# ROS2 消息类型
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# 升级后的 GraspPoint（字段均为可变长数组）
from my_bridge_pkg.msg import GraspPoint

# YOLOv8
from ultralytics import YOLO
import torch

import numpy as np
import cv2
import time

# 同时订阅多个话题并做时间同步 (message_filters 需在ROS上安装)
from message_filters import Subscriber, ApproximateTimeSynchronizer

import sys
import os

# ========== GG-CNN 部分(假设GG-CNN仓库路径) ==========
ggcnn_path = os.path.join(os.path.expanduser('~'), 'sagittarius_ws-main', 'src', 'ggcnn')
sys.path.append(ggcnn_path)
from models.ggcnn2 import GGCNN2_U_Net_MHSA_Upgraded


def normalise_rgb(rgb_img):
    """
    参考 Jacquard 的 Image.normalise():
    1) 转 float32 并 /255 => [0..1]
    2) 减去图像均值 => 零中心化
    """
    rgb_img = rgb_img.astype(np.float32)
    rgb_img /= 255.0
    mean_val = rgb_img.mean()
    rgb_img -= mean_val
    return rgb_img


def normalise_depth(depth_img):
    """
    参考 Jacquard 的 DepthImage.normalise():
    1) depth_img - mean
    2) clip到 [-1,1]
    """
    mean_val = depth_img.mean()
    depth_img = depth_img - mean_val
    np.clip(depth_img, -1.0, 1.0, out=depth_img)
    return depth_img


class YoloInfer3DNode(Node):
    def __init__(self):
        super().__init__('yolo_infer_3d_node')
        self.get_logger().info("Initializing YoloInfer3DNode...")

        # A) CVBridge
        self.bridge = CvBridge()

        # B) 声明订阅者 + 时间同步 (message_filters)
        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_rect_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.info_sub  = Subscriber(self, CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info')

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)

        # C) 加载 YOLOv8 分割模型
        self.model = YOLO('yolov8x-seg.pt')
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info("Using GPU for YOLO segment inference.")
        else:
            self.get_logger().info("GPU not available, using CPU for YOLO segment.")

        # D) 加载 GGCNN (4通道: RGB+Depth)
        self.ggcnn_model = GGCNN2_U_Net_MHSA_Upgraded(
            input_channels=4,
            base_ch=32,
            use_sigmoid_pos=False,
            use_cbam=True,
            use_mhsa=True,
            mhsa_heads=4,
            act_type='silu',
            use_shallow_fusion=True
        )
        # 根据实际路径修改
        weights_file = os.path.join(
            ggcnn_path,
            'output',
            'models',
            '250320_2114_ggcnn2_cbam_highperf_run',
            'epoch_109_iou_0.78_statedict.pt'
        )
        self.ggcnn_model.load_state_dict(torch.load(weights_file, map_location='cpu'))
        self.ggcnn_model.eval()
        if torch.cuda.is_available():
            self.ggcnn_model.to('cuda')
            self.get_logger().info("Using GPU for GG-CNN inference.")
        else:
            self.get_logger().info("GPU not available, using CPU for GG-CNN.")

        # E) 深度缩放比例 (假设原始深度单位为 mm => * 0.001 => m)
        self.depth_scale = 0.001

        # F) GGCNN输入尺寸
        self.ggcnn_input_size = 320

        # G) 发布者: /yolov8_infer/grasp_point (数组型)
        from sensor_msgs.msg import Image as RosImage
        self.grasp_pub = self.create_publisher(GraspPoint, '/yolov8_infer/grasp_point', 10)
        self.annotated_img_pub = self.create_publisher(RosImage, '/annotated_image', 10)

        # H) 帧率控制 (10FPS => 每隔0.1秒处理一次)
        self.last_process_time = self.get_clock().now()
        self.desired_period = 0.1  # 10FPS

        self.get_logger().info("YoloInfer3DNode initialized successfully.")

    def get_mean_depth_3x3(self, depth_img, cx, cy):
        """
        返回(cx,cy)为中心的3×3像素的非零平均深度
        """
        h, w = depth_img.shape[:2]
        half_size = 1
        x1 = max(0, cx - half_size)
        x2 = min(w, cx + half_size + 1)
        y1 = max(0, cy - half_size)
        y2 = min(h, cy + half_size + 1)

        roi = depth_img[y1:y2, x1:x2]
        roi_nonzero = roi[roi > 0]
        if len(roi_nonzero) == 0:
            return 0.0
        return float(np.mean(roi_nonzero))

    def get_class_color(self, cls_id):
        """
        根据类别ID生成固定随机种子，从而在每帧保持一致的颜色
        """
        np.random.seed(cls_id + 2023)
        color = np.random.rand(3) * 255
        return tuple(color.astype(np.uint8).tolist())  # (B, G, R)

    def sync_callback(self, color_msg, depth_msg, info_msg):
        now_time = self.get_clock().now()
        # 控制帧率: 10FPS
        if (now_time - self.last_process_time).nanoseconds < self.desired_period * 1e9:
            return
        self.last_process_time = now_time

        # 转OpenCV格式
        color_frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # 相机内参
        k = info_msg.k
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]

        # 原图分辨率
        h_orig, w_orig = color_frame.shape[:2]

        # 1) YOLO 分割推理
        results = self.model.predict(color_frame, conf=0.25)
        if len(results) == 0:
            # 若没检测到目标 => 发布原图 & 返回
            annotated_msg = self.bridge.cv2_to_imgmsg(color_frame, encoding="bgr8")
            self.annotated_img_pub.publish(annotated_msg)

            cv2.imshow("Seg + GGCNN", color_frame)
            cv2.waitKey(1)
            return

        result = results[0]
        boxes = result.boxes   # (N个目标)
        masks = result.masks   # (N个分割掩码)
        names = result.names   # 类别名称表

        # 做一个副本用于可视化
        annotated_frame = color_frame.copy()

        # ============ A) 分割可视化 ============
        if (masks is not None) and hasattr(masks, 'data'):
            mask_data = masks.data  # shape=[N, h_out, w_out]
            for i in range(mask_data.shape[0]):
                mask_i = mask_data[i].cpu().numpy()
                mask_resized = cv2.resize(
                    mask_i, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST
                )
                bin_mask = (mask_resized > 0.5)
                cls_id = int(boxes[i].cls[0])
                seg_color = self.get_class_color(cls_id)
                annotated_frame[bin_mask] = (
                    annotated_frame[bin_mask] * 0.5 + np.array(seg_color) * 0.5
                )

        if len(boxes) == 0:
            # 没有目标
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.annotated_img_pub.publish(annotated_msg)

            cv2.imshow("Seg + GGCNN", annotated_frame)
            cv2.waitKey(1)
            return

        # --- 准备收集所有目标到数组(纯Python类型) ---
        all_labels = []
        all_conf = []
        all_score = []
        all_angle = []
        all_width = []
        all_x = []
        all_y = []
        all_z = []
        all_frame = []

        best_q = -1.0
        best_idx = -1

        # ============ B) 遍历每个目标 => GGCNN ============
        for i, box in enumerate(boxes):
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])  # 转成Python float
            label = str(names[cls_id])  # 确保是str

            # 将 mask resize 回原图后 => boundingRect
            mask_i = masks.data[i].cpu().numpy()
            mask_resized = cv2.resize(mask_i, (w_orig, h_orig), interpolation=cv2.INTER_NEAREST)
            bin_mask = (mask_resized > 0.5).astype(np.uint8)

            x, y, bw, bh = cv2.boundingRect(bin_mask)
            if bw == 0 or bh == 0:
                continue

            # 可视化
            cv2.rectangle(annotated_frame, (x, y), (x+bw, y+bh), (0,255,0), 2)
            cv2.putText(
                annotated_frame, f"{label} {conf:.2f}",
                (x, max(y-10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2
            )

            # (1) 3×3 中心深度
            center_x = x + bw//2
            center_y = y + bh//2
            mean_d_3x3 = self.get_mean_depth_3x3(depth_frame, center_x, center_y)
            dist_m = float(mean_d_3x3 * self.depth_scale)
            if dist_m > 0.55 or dist_m <= 0:
                continue

            # (2) ROI大小限制
            if bw > self.ggcnn_input_size or bh > self.ggcnn_input_size:
                continue

            # 取ROI
            roi_color = color_frame[y:y+bh, x:x+bw].copy()
            roi_depth = depth_frame[y:y+bh, x:x+bw].copy()
            mask_sub  = bin_mask[y:y+bh, x:x+bw]

            # (3) 掩码外背景 => 设成白色 / 大深度
            bg_indices = (mask_sub == 0)
            roi_color[bg_indices] = (255, 255, 255)
            roi_depth[bg_indices] = 2000  # 2.0m

            # (4) 不缩放 => 320x320
            pad_top    = (self.ggcnn_input_size - bh) // 2
            pad_bottom = self.ggcnn_input_size - bh - pad_top
            pad_left   = (self.ggcnn_input_size - bw) // 2
            pad_right  = self.ggcnn_input_size - bw - pad_left

            roi_color_padded = cv2.copyMakeBorder(
                roi_color, pad_top, pad_bottom, pad_left, pad_right,
                borderType=cv2.BORDER_CONSTANT, value=(255,255,255)
            )
            roi_depth_padded = cv2.copyMakeBorder(
                roi_depth, pad_top, pad_bottom, pad_left, pad_right,
                borderType=cv2.BORDER_CONSTANT, value=2000
            )

            # (5) 归一化
            roi_color_padded_rgb = cv2.cvtColor(roi_color_padded, cv2.COLOR_BGR2RGB)
            roi_color_padded_rgb = normalise_rgb(roi_color_padded_rgb)

            roi_depth_padded_f = roi_depth_padded.astype(np.float32) * self.depth_scale
            roi_depth_padded_f = normalise_depth(roi_depth_padded_f)

            color_tensor = torch.from_numpy(roi_color_padded_rgb).permute(2, 0, 1).float()
            depth_tensor = torch.from_numpy(roi_depth_padded_f).unsqueeze(0).float()

            roi_4ch = torch.cat([color_tensor, depth_tensor], dim=0).unsqueeze(0)
            if torch.cuda.is_available():
                roi_4ch = roi_4ch.cuda()

            # (6) GG-CNN 推理
            with torch.no_grad():
                pos_out, cos_out, sin_out, width_out = self.ggcnn_model(roi_4ch)

            pos_map   = pos_out[0,0].cpu().numpy()
            cos_map   = cos_out[0,0].cpu().numpy()
            sin_map   = sin_out[0,0].cpu().numpy()
            width_map = width_out[0,0].cpu().numpy()

            max_q_idx = np.unravel_index(np.argmax(pos_map), pos_map.shape)
            gy, gx = max_q_idx
            grasp_q = float(pos_map[gy, gx])  # 强转 float
            angle   = float(np.arctan2(sin_map[gy, gx], cos_map[gy, gx]))
            gwidth  = float(width_map[gy, gx])

            if grasp_q < 0.1:
                continue

            # (7) 回到原图坐标
            real_x_in_roi = gx - pad_left
            real_y_in_roi = gy - pad_top
            real_x = x + real_x_in_roi
            real_y = y + real_y_in_roi

            d_pt = 0.0
            if (0 <= real_x < w_orig and 0 <= real_y < h_orig):
                d_pt = depth_frame[real_y, real_x]
            if d_pt == 0:
                d_pt = mean_d_3x3
            dist_m2 = float(d_pt * self.depth_scale)
            if dist_m2 <= 0:
                continue

            # ========== 改动：先算"光学坐标" => 再转换到 "camera_link" (x前,y左,z上) ==========

            # 1) 原光学投影(假定 +Z前, +X右, +Y下)
            Gx_opt = (real_x - cx) * dist_m2 / fx
            Gy_opt = (real_y - cy) * dist_m2 / fy
            Gz_opt = dist_m2

            # 2) 光学系 => 机身系 (x前, y左, z上):
            #    x_link = +z_opt
            #    y_link = -x_opt
            #    z_link = -y_opt
            x_link =  Gz_opt
            y_link = -Gx_opt
            z_link = -Gy_opt

            gwidth_m = float(gwidth / 100.0)  # 若网络输出cm => 转米

            # 可视化
            cv2.drawMarker(
                annotated_frame, (real_x, real_y), (0,0,255),
                markerType=cv2.MARKER_TILTED_CROSS, markerSize=15, thickness=2
            )
            cv2.putText(
                annotated_frame,
                f"GQ={grasp_q:.2f}, A={np.degrees(angle):.1f}",
                (real_x, real_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1
            )

            # 收集到数组中 (python原生)
            all_labels.append(label)
            all_conf.append(conf)
            all_score.append(grasp_q)
            all_angle.append(angle)
            all_width.append(gwidth_m)
            all_x.append(float(x_link))
            all_y.append(float(y_link))
            all_z.append(float(z_link))
            all_frame.append("camera_link")

            # 统计最高评分
            if grasp_q > best_q:
                best_q = grasp_q
                best_idx = len(all_labels) - 1

        # 如果没有有效抓取 => 只发叠加图
        if len(all_labels) == 0:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.annotated_img_pub.publish(annotated_msg)
            cv2.imshow("Seg + GGCNN", annotated_frame)
            cv2.waitKey(1)
            return

        # 构建数组消息
        gp_msg = GraspPoint()
        gp_msg.object_label = all_labels
        gp_msg.confidence   = all_conf
        gp_msg.score        = all_score
        gp_msg.angle        = all_angle
        gp_msg.width        = all_width
        gp_msg.x            = all_x
        gp_msg.y            = all_y
        gp_msg.z            = all_z
        gp_msg.frame_id     = all_frame

        # (可选) 给最高分目标加个后缀
        if best_idx >= 0:
            best_label = gp_msg.object_label[best_idx]
            gp_msg.object_label[best_idx] = best_label + " [BEST]"

        # 发布数组消息
        self.grasp_pub.publish(gp_msg)

        # 发布带叠加可视化图像
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.annotated_img_pub.publish(annotated_msg)

        # OpenCV窗口
        cv2.imshow("Seg + GGCNN", annotated_frame)
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