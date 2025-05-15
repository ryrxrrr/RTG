#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
GGCNN2_U_Net_MHSA_Upgraded:
在此基础上改为仅在 H/16 (encoder 第4层/decoder 第4层) 启用 MHSA，
保留其他部分。
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as transforms
from torch.utils.data import DataLoader, Dataset
from torch.nn.utils import clip_grad_norm_
from ggcnn.DepthMapRandom3DRotation import DepthMapRandom3DRotation

# -------------------------------------------------
# 0. 常用激活函数获取函数
# -------------------------------------------------
def get_activation(act_type: str = 'relu'):
    act_type = act_type.lower()
    if act_type == 'relu':
        return nn.ReLU(inplace=True)
    elif act_type == 'silu':
        return nn.SiLU(inplace=True)
    elif act_type == 'gelu':
        return nn.GELU()
    else:
        raise ValueError(f"Unsupported activation type: {act_type}")


# -------------------------------------------------
# 1. CBAM 注意力模块
# -------------------------------------------------
class ChannelAttention(nn.Module):
    def __init__(self, in_planes, ratio=8):
        super(ChannelAttention, self).__init__()
        self.avg_pool = nn.AdaptiveAvgPool2d(1)
        self.max_pool = nn.AdaptiveMaxPool2d(1)

        self.shared_MLP = nn.Sequential(
            nn.Linear(in_planes, in_planes // ratio, bias=False),
            nn.ReLU(inplace=True),
            nn.Linear(in_planes // ratio, in_planes, bias=False)
        )
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        b, c, _, _ = x.size()
        avg_out = self.avg_pool(x).view(b, c)
        avg_out = self.shared_MLP(avg_out)

        max_out = self.max_pool(x).view(b, c)
        max_out = self.shared_MLP(max_out)

        out = avg_out + max_out
        out = self.sigmoid(out).view(b, c, 1, 1)
        return x * out


class SpatialAttention(nn.Module):
    def __init__(self, kernel_size=7):
        super(SpatialAttention, self).__init__()
        self.conv = nn.Conv2d(2, 1, kernel_size=kernel_size,
                              padding=kernel_size // 2, bias=False)
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        avg_out = torch.mean(x, dim=1, keepdim=True)  # (B,1,H,W)
        max_out, _ = torch.max(x, dim=1, keepdim=True)  # (B,1,H,W)
        x_cat = torch.cat([avg_out, max_out], dim=1)    # (B,2,H,W)
        x_spatial = self.conv(x_cat)
        x_spatial = self.sigmoid(x_spatial)
        return x * x_spatial


class CBAM(nn.Module):
    def __init__(self, in_planes, ratio=8, kernel_size=7):
        super(CBAM, self).__init__()
        self.channel_attention = ChannelAttention(in_planes, ratio=ratio)
        self.spatial_attention = SpatialAttention(kernel_size=kernel_size)

    def forward(self, x):
        out = self.channel_attention(x)
        out = self.spatial_attention(out)
        return out


# -------------------------------------------------
# 2. MHSA（多头自注意力）模块
# -------------------------------------------------
class MHSA(nn.Module):
    def __init__(self, in_dim, heads=4):
        super(MHSA, self).__init__()
        assert in_dim % heads == 0, "in_dim 必须能被 heads 整除"
        self.in_dim = in_dim
        self.heads = heads
        self.dim_head = in_dim // heads

        self.q = nn.Conv2d(in_dim, in_dim, kernel_size=1, bias=False)
        self.k = nn.Conv2d(in_dim, in_dim, kernel_size=1, bias=False)
        self.v = nn.Conv2d(in_dim, in_dim, kernel_size=1, bias=False)
        self.proj = nn.Conv2d(in_dim, in_dim, kernel_size=1, bias=False)
        self.softmax = nn.Softmax(dim=-1)

    def forward(self, x):
        B, C, H, W = x.shape

        # Q,K,V
        q = self.q(x)  # (B, C, H, W)
        k = self.k(x)
        v = self.v(x)

        # 多头展开
        q = q.reshape(B * self.heads, self.dim_head, H * W)
        k = k.reshape(B * self.heads, self.dim_head, H * W)
        v = v.reshape(B * self.heads, self.dim_head, H * W)

        # 注意力分数
        attn_scores = torch.bmm(q.transpose(1, 2), k)  # (B*heads, H*W, H*W)
        attn_scores = attn_scores / (self.dim_head ** 0.5)
        attn_scores = self.softmax(attn_scores)

        # 应用到 v
        out = torch.bmm(attn_scores, v.transpose(1, 2))  # (B*heads, H*W, dim_head)

        # reshape回原形状
        out = out.transpose(1, 2).reshape(B, C, H, W)
        out = self.proj(out)
        return out


# -------------------------------------------------
# 3. DoubleConvAttn：Double Conv -> CBAM -> MHSA
# -------------------------------------------------
class DoubleConvAttn(nn.Module):
    def __init__(self, in_ch, out_ch,
                 use_cbam=True, use_mhsa=False,
                 cbam_ratio=8, cbam_ks=7,
                 mhsa_heads=4,
                 act_type='silu'):
        super(DoubleConvAttn, self).__init__()
        self.activation = get_activation(act_type)

        self.conv1 = nn.Conv2d(in_ch, out_ch, kernel_size=3, padding=1)
        self.bn1 = nn.BatchNorm2d(out_ch)
        self.conv2 = nn.Conv2d(out_ch, out_ch, kernel_size=3, padding=1)
        self.bn2 = nn.BatchNorm2d(out_ch)

        # 注意力
        self.cbam = CBAM(out_ch, ratio=cbam_ratio, kernel_size=cbam_ks) if use_cbam else nn.Identity()
        self.mhsa = MHSA(out_ch, heads=mhsa_heads) if use_mhsa else nn.Identity()

    def forward(self, x):
        x = self.activation(self.bn1(self.conv1(x)))
        x = self.activation(self.bn2(self.conv2(x)))
        x = self.cbam(x)
        x = self.mhsa(x)
        return x


# -------------------------------------------------
# 4. HeadBlock: 多头输出
# -------------------------------------------------
class HeadBlock(nn.Module):
    def __init__(self, in_ch, out_ch, mid_ch=None, act_type='silu', final_act=None):
        super(HeadBlock, self).__init__()
        if mid_ch is None:
            mid_ch = in_ch // 2

        self.conv1 = nn.Conv2d(in_ch, mid_ch, kernel_size=3, padding=1)
        self.bn1 = nn.BatchNorm2d(mid_ch)
        self.act = get_activation(act_type)

        self.conv2 = nn.Conv2d(mid_ch, out_ch, kernel_size=1)
        self.final_act = None
        if final_act is not None:
            if final_act.lower() == 'sigmoid':
                self.final_act = nn.Sigmoid()
            elif final_act.lower() == 'tanh':
                self.final_act = nn.Tanh()
            else:
                raise ValueError(f"Unsupported final_act: {final_act}")

    def forward(self, x):
        x = self.act(self.bn1(self.conv1(x)))
        x = self.conv2(x)
        if self.final_act is not None:
            x = self.final_act(x)
        return x


# -------------------------------------------------
# 5. GGCNN2_U_Net_MHSA_Upgraded（修改到 H/16）
#    仅在 H/16 分辨率上启用 MHSA (conv4_0 处),
#    其他层全部 use_mhsa=False.
# -------------------------------------------------
class GGCNN2_U_Net_MHSA_Upgraded(nn.Module):
    """
    仅在 H/16(encoder 第4层/decoder 第4层) 启用 MHSA,
    其余所有层 use_mhsa = False.
    """
    def __init__(
        self,
        input_channels=1,
        base_ch=32,
        use_sigmoid_pos=True,
        use_cbam=True,
        use_mhsa=False,   # 若=True, 在 H/16启用 MHSA; =False则关闭
        mhsa_heads=4,
        act_type='silu',
        use_shallow_fusion=True
    ):
        super().__init__()
        self.use_sigmoid_pos = use_sigmoid_pos
        self.use_shallow_fusion = use_shallow_fusion
        self.base_ch = base_ch
        self.act_type = act_type

        # ------------ ENCODER ------------
        # conv0_0 (H×W) -> 关闭MHSA
        self.conv0_0 = DoubleConvAttn(
            in_ch=input_channels, out_ch=base_ch,
            use_cbam=use_cbam, use_mhsa=False,
            mhsa_heads=mhsa_heads, act_type=act_type
        )
        self.pool0 = nn.MaxPool2d(2)

        # conv1_0 (H/2×W/2) -> 关闭MHSA
        self.conv1_0 = DoubleConvAttn(
            in_ch=base_ch, out_ch=base_ch*2,
            use_cbam=use_cbam, use_mhsa=False,
            mhsa_heads=mhsa_heads, act_type=act_type
        )
        self.pool1 = nn.MaxPool2d(2)

        # conv2_0 (H/4×W/4) -> 关闭MHSA
        self.conv2_0 = DoubleConvAttn(
            in_ch=base_ch*2, out_ch=base_ch*4,
            use_cbam=use_cbam, use_mhsa=False,
            mhsa_heads=mhsa_heads, act_type=act_type
        )
        self.pool2 = nn.MaxPool2d(2)

        # conv3_0 (H/8×W/8) -> 强制关闭
        self.conv3_0 = DoubleConvAttn(
            in_ch=base_ch*4, out_ch=base_ch*8,
            use_cbam=use_cbam,
            use_mhsa=False,  # 与之前不同，改为 False
            mhsa_heads=mhsa_heads, act_type=act_type
        )
        self.pool3 = nn.MaxPool2d(2)

        # conv4_0 (H/16×W/16) -> 根据 use_mhsa 决定
        self.conv4_0 = DoubleConvAttn(
            in_ch=base_ch*8, out_ch=base_ch*16,
            use_cbam=use_cbam, 
            use_mhsa=use_mhsa,  # 仅此处
            mhsa_heads=mhsa_heads, act_type=act_type
        )

        # ------------ DECODER ------------
        # step=1 (conv0_1, conv1_1, conv2_1, conv3_1)
        self.conv0_1 = DoubleConvAttn(
            in_ch=base_ch + base_ch*2, out_ch=base_ch,
            use_cbam=use_cbam, use_mhsa=False,
            mhsa_heads=mhsa_heads, act_type=act_type
        )
        self.conv1_1 = DoubleConvAttn(
            in_ch=base_ch*2 + base_ch*4, out_ch=base_ch*2,
            use_cbam=use_cbam, use_mhsa=False,
            mhsa_heads=mhsa_heads, act_type=act_type
        )
        self.conv2_1 = DoubleConvAttn(
            in_ch=base_ch*4 + base_ch*8, out_ch=base_ch*4,
            use_cbam=use_cbam, use_mhsa=False,
            mhsa_heads=mhsa_heads, act_type=act_type
        )
        self.conv3_1 = DoubleConvAttn(
            in_ch=base_ch*8 + base_ch*16, out_ch=base_ch*8,
            use_cbam=use_cbam,
            use_mhsa=False,  # 与之前不同，改为 False
            mhsa_heads=mhsa_heads, act_type=act_type
        )

        # step=2 (conv0_2, conv1_2, conv2_2)
        self.conv0_2 = DoubleConvAttn(
            in_ch=base_ch + base_ch*2, out_ch=base_ch,
            use_cbam=use_cbam, use_mhsa=False,
            mhsa_heads=mhsa_heads, act_type=act_type
        )
        self.conv1_2 = DoubleConvAttn(
            in_ch=base_ch*2 + base_ch*4, out_ch=base_ch*2,
            use_cbam=use_cbam, use_mhsa=False,
            mhsa_heads=mhsa_heads, act_type=act_type
        )
        self.conv2_2 = DoubleConvAttn(
            in_ch=base_ch*4 + base_ch*8, out_ch=base_ch*4,
            use_cbam=use_cbam, use_mhsa=False,
            mhsa_heads=mhsa_heads, act_type=act_type
        )

        # step=3 (conv0_3, conv1_3)
        self.conv0_3 = DoubleConvAttn(
            in_ch=base_ch + base_ch*2, out_ch=base_ch,
            use_cbam=use_cbam, use_mhsa=False,
            mhsa_heads=mhsa_heads, act_type=act_type
        )
        self.conv1_3 = DoubleConvAttn(
            in_ch=base_ch*2 + base_ch*4, out_ch=base_ch*2,
            use_cbam=use_cbam, use_mhsa=False,
            mhsa_heads=mhsa_heads, act_type=act_type
        )

        # step=4 (conv0_4)
        self.conv0_4 = DoubleConvAttn(
            in_ch=base_ch + base_ch*2, out_ch=base_ch,
            use_cbam=use_cbam, use_mhsa=False,
            mhsa_heads=mhsa_heads, act_type=act_type
        )

        # 上采样
        self.upsample = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)

        # 浅层融合
        if use_shallow_fusion:
            self.fusion_conv = nn.Sequential(
                nn.Conv2d(base_ch + base_ch, base_ch, kernel_size=1, bias=False),
                nn.BatchNorm2d(base_ch),
                get_activation(act_type)
            )
        else:
            self.fusion_conv = nn.Identity()

        # 输出头
        final_act_pos = 'sigmoid' if use_sigmoid_pos else None
        self.pos_head = HeadBlock(base_ch, 1, act_type=act_type, final_act=final_act_pos)
        self.cos_head = HeadBlock(base_ch, 1, act_type=act_type)
        self.sin_head = HeadBlock(base_ch, 1, act_type=act_type)
        self.width_head = HeadBlock(base_ch, 1, act_type=act_type)

        self._init_params()

    def _init_params(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)

    def forward(self, x):
        # --------- Encoder ---------
        X_0_0 = self.conv0_0(x)                   
        X_1_0 = self.conv1_0(self.pool0(X_0_0))   
        X_2_0 = self.conv2_0(self.pool1(X_1_0))   
        X_3_0 = self.conv3_0(self.pool2(X_2_0))   
        X_4_0 = self.conv4_0(self.pool3(X_3_0))   # 仅在这里启用 MHSA

        # --------- Decoder (nested) ---------
        # step=1
        X_0_1 = self.conv0_1(torch.cat([X_0_0, self.upsample(X_1_0)], dim=1))
        X_1_1 = self.conv1_1(torch.cat([X_1_0, self.upsample(X_2_0)], dim=1))
        X_2_1 = self.conv2_1(torch.cat([X_2_0, self.upsample(X_3_0)], dim=1))
        X_3_1 = self.conv3_1(torch.cat([X_3_0, self.upsample(X_4_0)], dim=1))

        # step=2
        X_0_2 = self.conv0_2(torch.cat([X_0_1, self.upsample(X_1_1)], dim=1))
        X_1_2 = self.conv1_2(torch.cat([X_1_1, self.upsample(X_2_1)], dim=1))
        X_2_2 = self.conv2_2(torch.cat([X_2_1, self.upsample(X_3_1)], dim=1))

        # step=3
        X_0_3 = self.conv0_3(torch.cat([X_0_2, self.upsample(X_1_2)], dim=1))
        X_1_3 = self.conv1_3(torch.cat([X_1_2, self.upsample(X_2_2)], dim=1))

        # step=4
        X_0_4 = self.conv0_4(torch.cat([X_0_3, self.upsample(X_1_3)], dim=1))

        X_out = X_0_4

        # 浅层融合
        if self.use_shallow_fusion:
            X_0_0_up = F.interpolate(X_0_0, size=X_out.shape[2:], mode='bilinear', align_corners=True)
            X_out = torch.cat([X_out, X_0_0_up], dim=1)
            X_out = self.fusion_conv(X_out)

        # 输出
        pos_out = self.pos_head(X_out)
        cos_out = self.cos_head(X_out)
        sin_out = self.sin_head(X_out)
        width_out = self.width_head(X_out)

        return pos_out, cos_out, sin_out, width_out

    def compute_loss(self, input_data, target_data,
                     w_pos=1.0, w_cos_sin=1.0, w_width=1.0):
        y_pos, y_cos, y_sin, y_width = target_data
        pos_pred, cos_pred, sin_pred, width_pred = self.forward(input_data)

        p_loss     = F.mse_loss(pos_pred, y_pos)
        cos_loss   = F.mse_loss(cos_pred, y_cos)
        sin_loss   = F.mse_loss(sin_pred, y_sin)
        width_loss = F.mse_loss(width_pred, y_width)

        total_loss = (w_pos * p_loss) \
                     + (w_cos_sin * (cos_loss + sin_loss)) \
                     + (w_width * width_loss)

        return {
            'loss': total_loss,
            'losses': {
                'p_loss': p_loss,
                'cos_loss': cos_loss,
                'sin_loss': sin_loss,
                'width_loss': width_loss
            },
            'pred': {
                'pos': pos_pred,
                'cos': cos_pred,
                'sin': sin_pred,
                'width': width_pred
            }
        }
