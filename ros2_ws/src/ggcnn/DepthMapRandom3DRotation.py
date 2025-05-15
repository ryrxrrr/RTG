import torch
import numpy as np
import random
import math

class DepthMapRandom3DRotation:
    """
    结合3D旋转 + 随机数据遗失 + 随机过度曝光 + 噪声 / 散射效应 + 多光源环境影响 的深度图增强。
    1) 深度图->点云->随机三轴旋转->投影回深度图
    2) 随机遗失：将局部区域设为0
    3) 随机过曝：局部区域设为过小/过大(示例以乘法放大或置0)
    4) 随机传感器噪声：模拟ToF / 结构光测量中的散射或测量偏差
    5) 随机多光源 / 环境光：以局部增减深度值或叠加噪声方式来模拟
    """
    def __init__(self, 
                 fx=431.0631103515625, 
                 fy=430.7428283691406, 
                 cx=420.021728515625, 
                 cy=239.2523956298828,
                 # Occlusion相关
                 occlusion_prob=0.3,       # 有多大概率执行"随机遗失"操作
                 max_occlusion_patches=2,  # 最多生成多少个遗失矩形
                 max_occlusion_size=40,    # 每个遗失矩形最长边最大像素
                 # Overexposure相关
                 overexp_prob=0.3,         # 有多大概率执行"过度曝光"操作
                 max_overexp_patches=2,    # 最多生成多少个曝光矩形
                 max_overexp_size=40,      # 每个曝光矩形最长边最大像素
                 overexp_mode='random',    # 'brighten'(乘法放大) 或 'zero'(置0) 或 'random'
                 # 传感器噪声相关
                 sensor_noise_prob=0.5,    # 有多大概率执行随机传感器噪声
                 base_noise_level=0.01,    # 基础噪声标准差(可根据实际调参)
                 depth_factor=0.01,        # 噪声随深度增加的系数
                 # 多光源 / 环境光
                 light_effect_prob=0.3,    # 有多大概率执行多光源环境光模拟
                 max_light_patches=2,      # 最多生成多少个光照区域
                 max_light_size=40,        # 每个区域最长边最大像素
                 light_mode='random',      # 'bias'(加减常量) / 'noise'(叠加噪声) / 'random'
                 light_bias_range=(-0.05, 0.05),   # 加减常量的范围（可调节）
                 light_noise_std=0.02,            # 在此范围内给局部区域额外叠加噪声
                 ):
        """
        参数说明:
        - fx, fy, cx, cy: 相机内参 (焦距/主点坐标), 用于深度->点云->深度的转换.
        - occlusion_prob: 执行随机遗失的概率 (0~1之间).
        - max_occlusion_patches, max_occlusion_size: 遗失矩形数量与最大尺寸.
        - overexp_prob, overexp_mode: 执行随机过曝的概率和过曝方式.
        - sensor_noise_prob: 执行随机传感器噪声的概率.
        - base_noise_level: 基础噪声量；后面会作为正态分布的sigma起点.
        - depth_factor: 噪声随深度值增加的系数 (深度越大噪声可能越明显).
        - light_effect_prob: 执行多光源/环境光模拟的概率.
        - max_light_patches, max_light_size: 多光源影响区域数量与最大尺寸.
        - light_mode: 'bias'表示给区域加减固定量, 'noise'表示叠加额外噪声, 'random'随机二者.
        - light_bias_range: 在bias模式下，加或减的偏移量区间 (例如 -0.05~0.05).
        - light_noise_std: 如果是噪声模式，则叠加的正态分布噪声标准差.
        """
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

        # 随机遗失相关
        self.occlusion_prob = occlusion_prob
        self.max_occ_patches = max_occlusion_patches
        self.max_occ_size = max_occlusion_size

        # 随机过曝相关
        self.overexp_prob = overexp_prob
        self.max_oe_patches = max_overexp_patches
        self.max_oe_size = max_overexp_size
        self.overexp_mode = overexp_mode

        # 传感器噪声相关
        self.sensor_noise_prob = sensor_noise_prob
        self.base_noise_level = base_noise_level
        self.depth_factor = depth_factor

        # 多光源 / 环境光相关
        self.light_effect_prob = light_effect_prob
        self.max_light_patches = max_light_patches
        self.max_light_size = max_light_size
        self.light_mode = light_mode
        self.light_bias_range = light_bias_range
        self.light_noise_std = light_noise_std


    def __call__(self, depth_tensor):
        """
        depth_tensor: Tensor, shape = (1, H, W), 单通道深度图
        returns: 增强后的深度图 (Tensor, shape = (1, H, W))
        """
        assert depth_tensor.ndim == 3 and depth_tensor.shape[0] == 1, \
            "输入必须是 (1, H, W) 的单通道深度图 Tensor."

        depth_np = depth_tensor.squeeze(0).cpu().numpy()  # (H, W)

        # 1) depth -> pointcloud
        pc = self.depth_to_pointcloud(depth_np)

        # 2) 随机三轴旋转角度 (X、Y±3°, Z±90°)
        angle_x_deg = random.uniform(-3, 3)  
        angle_y_deg = random.uniform(-3, 3)  
        angle_z_deg = random.uniform(-90, 90)  

        angle_x = math.radians(angle_x_deg)
        angle_y = math.radians(angle_y_deg)
        angle_z = math.radians(angle_z_deg)

        # 3) 旋转点云
        pc_rot = self.rotate_pointcloud(pc, angle_x, angle_y, angle_z)

        # 4) 投影回深度图
        new_depth = self.pointcloud_to_depth(pc_rot, depth_np.shape)

        # 5) 随机执行数据遗失 (occlusion)
        if random.random() < self.occlusion_prob:
            new_depth = self.random_occlusion(new_depth)

        # 6) 随机执行过度曝光 (overexposure)
        if random.random() < self.overexp_prob:
            new_depth = self.random_overexposure(new_depth)

        # 7) 随机执行传感器噪声 / 散射效应
        if random.random() < self.sensor_noise_prob:
            new_depth = self.random_sensor_noise(new_depth)

        # 8) 随机执行多光源 / 环境光模拟
        if random.random() < self.light_effect_prob:
            new_depth = self.random_light_effect(new_depth)

        # 转回 Tensor
        new_depth_tensor = torch.from_numpy(new_depth).unsqueeze(0).float()
        return new_depth_tensor

    # -------------------------------
    # A. 辅助函数: 深度图 -> 点云
    # -------------------------------
    def depth_to_pointcloud(self, depth_img):
        """ 将深度图转换为点云 (N,3). """
        H, W = depth_img.shape
        xs = np.arange(W)
        ys = np.arange(H)
        u, v = np.meshgrid(xs, ys)  # shape=(H,W)

        Z = depth_img.reshape(-1)  # (H*W,)
        X = (u.reshape(-1) - self.cx) * Z / self.fx
        Y = (v.reshape(-1) - self.cy) * Z / self.fy

        pc = np.stack([X, Y, Z], axis=-1)  # (H*W, 3)
        return pc

    # -------------------------------
    # B. 辅助函数: 三轴旋转点云
    # -------------------------------
    def rotate_pointcloud(self, pc, angle_x, angle_y, angle_z):
        """
        绕 x轴, y轴, z轴分别旋转 angle_x, angle_y, angle_z.
        """
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(angle_x), -math.sin(angle_x)],
            [0, math.sin(angle_x),  math.cos(angle_x)]
        ], dtype=np.float32)

        Ry = np.array([
            [ math.cos(angle_y), 0, math.sin(angle_y)],
            [0,                 1, 0],
            [-math.sin(angle_y),0, math.cos(angle_y)]
        ], dtype=np.float32)

        Rz = np.array([
            [math.cos(angle_z), -math.sin(angle_z), 0],
            [math.sin(angle_z),  math.cos(angle_z), 0],
            [0,                 0,                  1]
        ], dtype=np.float32)

        # 先绕 x, 再 y, 再 z (可根据需求更改顺序)
        R = Rz @ Ry @ Rx
        pc_rot = pc @ R.T
        return pc_rot

    # -------------------------------
    # C. 辅助函数: 点云 -> 深度图
    # -------------------------------
    def pointcloud_to_depth(self, pc, shape):
        """
        将点云投影回 shape(H,W) 的深度图.
        若有重叠/遮挡, 简易取最小Z(离相机最近).
        """
        H, W = shape
        X = pc[:, 0]
        Y = pc[:, 1]
        Z = pc[:, 2]

        # 只投影 Z>0 的点(在相机前方)
        valid_mask = Z > 1e-6
        X = X[valid_mask]
        Y = Y[valid_mask]
        Z = Z[valid_mask]

        u = (self.fx * X / Z + self.cx).astype(np.int32)
        v = (self.fy * Y / Z + self.cy).astype(np.int32)

        valid_uv = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        u = u[valid_uv]
        v = v[valid_uv]
        Z = Z[valid_uv]

        new_depth = np.zeros((H, W), dtype=np.float32)

        # 如果同一像素有多个点投影, 选最小Z
        for px, py, pz in zip(u, v, Z):
            if new_depth[py, px] == 0:
                new_depth[py, px] = pz
            else:
                new_depth[py, px] = min(new_depth[py, px], pz)

        return new_depth

    # -----------------------------------
    # D. 随机数据遗失(occlusion)函数
    # -----------------------------------
    def random_occlusion(self, depth_img):
        """
        在深度图上随机生成若干矩形区域, 将其值设为0, 
        模拟传感器遮挡/丢失等情况。
        """
        H, W = depth_img.shape
        num_patches = random.randint(1, self.max_occ_patches)
        
        for _ in range(num_patches):
            # 随机矩形大小
            occ_h = random.randint(5, self.max_occ_size)
            occ_w = random.randint(5, self.max_occ_size)
            # 随机左上角位置
            top = random.randint(0, max(0, H - occ_h))
            left = random.randint(0, max(0, W - occ_w))
            # 将该区域置0
            depth_img[top:top+occ_h, left:left+occ_w] = 0
        
        return depth_img

    # -----------------------------------
    # E. 随机过度曝光(overexposure)函数
    # -----------------------------------
    def random_overexposure(self, depth_img):
        """
        在深度图上随机生成若干矩形区域, 对其进行过曝处理:
        - 方式1: 乘以 (1.5~2.0), 使值变大
        - 方式2: 直接置0
        - 'random' 模式下随机选择1或2
        """
        H, W = depth_img.shape
        num_patches = random.randint(1, self.max_oe_patches)

        for _ in range(num_patches):
            oe_h = random.randint(5, self.max_oe_size)
            oe_w = random.randint(5, self.max_oe_size)
            top = random.randint(0, max(0, H - oe_h))
            left = random.randint(0, max(0, W - oe_w))

            patch = depth_img[top:top+oe_h, left:left+oe_w]

            mode = self.overexp_mode
            if mode == 'random':
                mode = random.choice(['brighten', 'zero'])
            
            if mode == 'brighten':
                scale_factor = random.uniform(1.5, 2.0)
                patch *= scale_factor
            elif mode == 'zero':
                patch[:] = 0
            else:
                # 如果用户指定了别的就默认置0
                patch[:] = 0

            depth_img[top:top+oe_h, left:left+oe_w] = patch

        return depth_img

    # -----------------------------------
    # F. 随机传感器噪声 (模拟ToF / 结构光散射)
    # -----------------------------------
    def random_sensor_noise(self, depth_img):
        """
        给深度图整体添加随机噪声(可随深度值而变化)，
        以模拟传感器测量中的散射或噪声效应。

        示例实现：对非零深度的像素添加 ~ N(0, sigma^2) 的噪声，
        其中 sigma = base_noise_level + depth_factor * depth
        """
        H, W = depth_img.shape

        # 只对有效深度(>0)区域添加噪声
        valid_mask = depth_img > 0
        depth_valid = depth_img[valid_mask]

        # 计算噪声标准差
        # 如果想让噪声随深度增大，可以做以下操作:
        noise_std = self.base_noise_level + self.depth_factor * depth_valid

        # 根据每个像素的 noise_std 来采样噪声
        noise = np.random.randn(depth_valid.size).astype(np.float32)  # 标准正态
        noise = noise * noise_std  # 缩放到对应像素的std

        # 将噪声叠加回去
        depth_valid_noisy = depth_valid + noise
        # 避免出现负值
        depth_valid_noisy = np.clip(depth_valid_noisy, 0, None)

        # 写回 depth_img
        depth_img[valid_mask] = depth_valid_noisy
        
        return depth_img

    # -----------------------------------
    # G. 随机多光源 / 环境光影响
    # -----------------------------------
    def random_light_effect(self, depth_img):
        """
        随机在深度图上生成若干区域，
        并对这些区域的深度值进行加减偏移或叠加噪声，
        用于模拟多光源/环境光对测量的干扰或偏差。
        """
        H, W = depth_img.shape
        num_patches = random.randint(1, self.max_light_patches)

        for _ in range(num_patches):
            # 随机区域大小
            lh = random.randint(5, self.max_light_size)
            lw = random.randint(5, self.max_light_size)
            top = random.randint(0, max(0, H - lh))
            left = random.randint(0, max(0, W - lw))

            patch = depth_img[top:top+lh, left:left+lw]

            # 确定本次使用的模式
            mode = self.light_mode
            if mode == 'random':
                mode = random.choice(['bias', 'noise'])

            if mode == 'bias':
                # 在指定范围内随机一个偏移值
                bias = random.uniform(self.light_bias_range[0], self.light_bias_range[1])
                # 加上偏移量：可正可负
                # 注意：深度为距离，不宜出现负值，要做clip
                patch += bias
                patch[:] = np.clip(patch, 0, None)
            else:
                # noise模式：给该区域再叠加一次噪声
                # 也可结合depth做std变化
                local_noise = np.random.normal(loc=0.0, scale=self.light_noise_std, size=patch.shape)
                patch += local_noise
                patch[:] = np.clip(patch, 0, None)

            depth_img[top:top+lh, left:left+lw] = patch

        return depth_img
