import numpy as np
import matplotlib.pyplot as plt
from skimage.draw import polygon
from skimage.feature import peak_local_max


def _gr_text_to_no(l, offset=(0, 0)):
    """
    Transform a single point from a Cornell file line to a pair of ints.
    :param l: Line from Cornell grasp file (str)
    :param offset: Offset to apply to point positions
    :return: Point [y, x]
    """
    x, y = l.split()
    return [int(round(float(y))) - offset[0], int(round(float(x))) - offset[1]]


class GraspRectangles:
    """
    Convenience class for loading and operating on sets of Grasp Rectangles.
    """
    def __init__(self, grs=None):
        if grs:
            self.grs = grs
        else:
            self.grs = []

    def __getitem__(self, item):
        return self.grs[item]

    def __iter__(self):
        return self.grs.__iter__()

    def __getattr__(self, attr):
        """
        如果 GraspRectangle 有某个函数，就把它映射到内部所有的 grasp rectangles 上。
        """
        if hasattr(GraspRectangle, attr) and callable(getattr(GraspRectangle, attr)):
            return lambda *args, **kwargs: list(map(lambda gr: getattr(gr, attr)(*args, **kwargs), self.grs))
        else:
            raise AttributeError(f"Couldn't find function {attr} in GraspRectangles or GraspRectangle")

    @classmethod
    def load_from_array(cls, arr):
        """
        从 Nx4x2 的数组中加载多个 GraspRectangle。
        :param arr: Nx4x2 array
        :return: GraspRectangles
        """
        grs = []
        for i in range(arr.shape[0]):
            grp = arr[i, :, :].squeeze()
            if grp.max() == 0:
                # 可能是零填充
                break
            grs.append(GraspRectangle(grp))
        return cls(grs)

    @classmethod
    def load_from_cornell_file(cls, fname):
        """
        从 Cornell grasp dataset 的标签文件中加载多个 GraspRectangles。
        :param fname: 文本文件路径
        :return: GraspRectangles
        """
        grs = []
        with open(fname) as f:
            while True:
                # 4 行一组
                p0 = f.readline()
                if not p0:
                    break  # EOF
                p1, p2, p3 = f.readline(), f.readline(), f.readline()
                try:
                    gr = np.array([
                        _gr_text_to_no(p0),
                        _gr_text_to_no(p1),
                        _gr_text_to_no(p2),
                        _gr_text_to_no(p3)
                    ])
                    grs.append(GraspRectangle(gr))
                except ValueError:
                    # 某些文件可能存在异常数据
                    continue
        return cls(grs)

    @classmethod
    def load_from_jacquard_file(cls, fname, scale=1.0):
        """
        从 Jacquard 数据集文件中加载 GraspRectangles。
        :param fname: 标签文件路径
        :param scale: 缩放因子
        :return: GraspRectangles
        """
        grs = []
        with open(fname) as f:
            for l in f:
                x, y, theta, w, h = [float(v) for v in l.strip().split(';')]
                # Jacquard 文件里: 行/列 => (y, x)，并且角度方向相反
                gr_obj = Grasp(np.array([y, x]), -theta / 180.0 * np.pi, w, h)
                grs.append(gr_obj.as_gr)
        grs = cls(grs)
        grs.scale(scale)
        return grs

    def append(self, gr):
        """
        添加一个单独的 GraspRectangle。
        """
        self.grs.append(gr)

    def copy(self):
        """
        深拷贝
        """
        new_grs = GraspRectangles()
        for gr in self.grs:
            new_grs.append(gr.copy())
        return new_grs

    def show(self, ax=None, shape=None):
        """
        可视化显示所有的 GraspRectangles
        """
        if ax is None:
            f = plt.figure()
            ax = f.add_subplot(1, 1, 1)
            ax.imshow(np.zeros(shape))
            ax.axis([0, shape[1], shape[0], 0])
            self.plot(ax)
            plt.show()
        else:
            self.plot(ax)

    def draw(self, shape, position=True, angle=True, width=True):
        """
        画出所有 GraspRectangles 所形成的“掩码”图，用作网络训练数据等。
        :param shape: (H, W)
        :param position: 如果 True，则输出 Q 图
        :param angle: 如果 True，则输出 angle 图
        :param width: 如果 True，则输出 width 图
        :return: (pos_out, ang_out, width_out)
        """
        if position:
            pos_out = np.zeros(shape)
        else:
            pos_out = None

        if angle:
            ang_out = np.zeros(shape)
        else:
            ang_out = None

        if width:
            width_out = np.zeros(shape)
        else:
            width_out = None

        for gr in self.grs:
            rr, cc = gr.compact_polygon_coords(shape)
            if position:
                pos_out[rr, cc] = 1.0
            if angle:
                ang_out[rr, cc] = gr.angle
            if width:
                width_out[rr, cc] = gr.length

        return pos_out, ang_out, width_out

    def to_array(self, pad_to=0):
        """
        将所有 GraspRectangles 转成一个 Nx4x2 数组
        :param pad_to: 如果指定，则补 0 到对应长度
        """
        a = np.stack([gr.points for gr in self.grs])
        if pad_to:
            if pad_to > len(self.grs):
                a = np.concatenate((a, np.zeros((pad_to - len(self.grs), 4, 2))))
        return a.astype(int)

    @property
    def center(self):
        """
        返回所有抓取矩形的平均中心
        """
        points = [gr.points for gr in self.grs]
        return np.mean(np.vstack(points), axis=0).astype(int)


class GraspRectangle:
    """
    表示单个抓取矩形：四个顶点的 (y, x) 坐标
    """
    def __init__(self, points):
        # 确保 points 至少是 (N,2)
        self.points = points

    def __str__(self):
        return str(self.points)

    @property
    def angle(self):
        """
        返回抓取与水平轴的夹角 (radians)
        """
        dx = self.points[1, 1] - self.points[0, 1]
        dy = self.points[1, 0] - self.points[0, 0]
        # 通过 arctan2 计算角度, 并保证结果在 [-π/2, π/2)
        return (np.arctan2(-dy, dx) + np.pi/2) % np.pi - np.pi/2

    @property
    def as_grasp(self):
        """
        转化为 Grasp 对象
        """
        return Grasp(self.center, self.angle, self.length, self.width)

    @property
    def center(self):
        """
        矩形四个角点的平均值即中心
        """
        return self.points.mean(axis=0).astype(int)

    @property
    def length(self):
        """
        抓取的长度（沿抓取方向的边）
        """
        dx = self.points[1, 1] - self.points[0, 1]
        dy = self.points[1, 0] - self.points[0, 0]
        return np.sqrt(dx**2 + dy**2)

    @property
    def width(self):
        """
        抓取的宽度（与抓取方向垂直的边）
        """
        dy = self.points[2, 1] - self.points[1, 1]
        dx = self.points[2, 0] - self.points[1, 0]
        return np.sqrt(dx**2 + dy**2)

    def polygon_coords(self, shape=None):
        """
        返回该抓取矩形的多边形内像素坐标 (row, col)
        """
        return polygon(self.points[:, 0], self.points[:, 1], shape)

    def compact_polygon_coords(self, shape=None):
        """
        返回该抓取矩形中间三分之一区域（仅中间那段）的多边形像素坐标 (row, col)。
        """
        return Grasp(self.center, self.angle, self.length/3, self.width).as_gr.polygon_coords(shape)

    def iou(self, gr, angle_threshold=np.pi/18):
        """
        与另一个 GraspRectangle 的 IoU（仅在角度差小于 angle_threshold 时计算，否则返回 0）
        """
        # 如果角度相差太大，直接视为 0
        if abs((self.angle - gr.angle + np.pi/2) % np.pi - np.pi/2) > angle_threshold:
            return 0

        rr1, cc1 = self.polygon_coords()
        rr2, cc2 = polygon(gr.points[:, 0], gr.points[:, 1])

        try:
            r_max = max(rr1.max(), rr2.max()) + 1
            c_max = max(cc1.max(), cc2.max()) + 1
        except ValueError:
            # 万一其中一个是空的坐标
            return 0

        canvas = np.zeros((r_max, c_max), dtype=np.uint8)
        canvas[rr1, cc1] += 1
        canvas[rr2, cc2] += 1

        union = np.sum(canvas > 0)
        if union == 0:
            return 0
        intersection = np.sum(canvas == 2)
        return intersection / union

    def copy(self):
        """
        :return: 返回副本
        """
        return GraspRectangle(self.points.copy())

    def offset(self, offset):
        """
        整体平移
        """
        # 若你想彻底不打印，可以注释掉
        # print(f"[DEBUG] offset() called with offset={offset}, shape={np.shape(offset)}")
        self.points += np.array(offset).reshape((1, 2))

    def rotate(self, angle, center):
        """
        绕某个中心点旋转
        :param angle: 旋转角（弧度）
        :param center: 旋转中心 (y, x)
        """
        # 若你想彻底不打印，可以注释掉
        # print(f"[DEBUG] rotate() called with angle={angle}, center={center}")
        angle = float(angle)  # 确保是标量
        R = np.array([
            [np.cos(-angle),  np.sin(-angle)],
            [-np.sin(-angle), np.cos(-angle)],
        ])
        c = np.array(center, dtype=float).reshape((1, 2))

        # 检查 self.points 形状是否 (N, 2)
        if len(self.points.shape) != 2 or self.points.shape[1] != 2:
            raise ValueError(f"Invalid shape for self.points. Expected (N,2), got {self.points.shape}.")

        self.points = ((R @ (self.points - c).T).T + c).astype(int)

    def scale(self, factor):
        """
        按照给定因子缩放
        """
        # 若你想彻底不打印，可以注释掉
        # print(f"[DEBUG] scale() called with factor={factor}, type={type(factor)}")
        if factor == 1.0:
            return
        factor = float(factor)
        self.points = (self.points * factor).astype(int)

    def plot(self, ax, color=None):
        """
        在指定 ax 上绘制该抓取矩形
        """
        points = np.vstack((self.points, self.points[0]))
        ax.plot(points[:, 1], points[:, 0], color=color)

    def zoom(self, factor, center):
        """
        以指定中心点进行放缩
        """
        # 若你想彻底不打印，可以注释掉
        # print(f"[DEBUG] zoom() called with factor={factor}, center={center}")
        factor = float(factor)
        T = np.array([
            [1.0/factor, 0.0],
            [0.0, 1.0/factor]
        ])
        c = np.array(center, dtype=float).reshape((1, 2))

        if len(self.points.shape) != 2 or self.points.shape[1] != 2:
            raise ValueError(f"Invalid shape for self.points. Expected (N,2), got {self.points.shape}.")

        self.points = ((T @ (self.points - c).T).T + c).astype(int)


class Grasp:
    """
    用中心点、旋转角、夹爪长度与宽度表征一个抓取
    """
    def __init__(self, center, angle, length=60, width=30):
        self.center = center
        # 正角度代表相对于水平方向逆时针旋转
        self.angle = angle
        self.length = length
        self.width = width

    @property
    def as_gr(self):
        """
        转化为 GraspRectangle
        """
        xo = np.cos(self.angle)
        yo = np.sin(self.angle)

        y1 = self.center[0] + self.length/2 * yo
        x1 = self.center[1] - self.length/2 * xo
        y2 = self.center[0] - self.length/2 * yo
        x2 = self.center[1] + self.length/2 * xo

        points = np.array([
            [y1 - self.width/2 * xo, x1 - self.width/2 * yo],
            [y2 - self.width/2 * xo, x2 - self.width/2 * yo],
            [y2 + self.width/2 * xo, x2 + self.width/2 * yo],
            [y1 + self.width/2 * xo, x1 + self.width/2 * yo],
        ], dtype=float)

        return GraspRectangle(points)

    def max_iou(self, grs):
        """
        计算本抓取与一批 GraspRectangle 的最大 IOU
        """
        self_gr = self.as_gr
        max_iou = 0.0
        for gr in grs:
            iou = self_gr.iou(gr)
            if iou > max_iou:
                max_iou = iou
        return max_iou

    def plot(self, ax, color=None):
        """
        可视化
        """
        self.as_gr.plot(ax, color)

    def to_jacquard(self, scale=1):
        """
        输出为 Jacquard 格式字符串
        x;y;theta;w;h
        """
        return '%0.2f;%0.2f;%0.2f;%0.2f;%0.2f' % (
            self.center[1]*scale,
            self.center[0]*scale,
            -1*self.angle*180/np.pi,
            self.length*scale,
            self.width*scale
        )


def detect_grasps(q_img, ang_img, width_img=None, no_grasps=1):
    """
    从 GG-CNN 的输出 Q, angle, width 中检测抓取
    :param q_img: Q图
    :param ang_img: angle图
    :param width_img: width图 (可选)
    :param no_grasps: 最多返回多少个候选抓取
    :return: list of Grasp
    """
    local_max = peak_local_max(q_img, min_distance=20, threshold_abs=0.2, num_peaks=no_grasps)

    grasps = []
    for grasp_point_array in local_max:
        grasp_point = tuple(grasp_point_array)
        grasp_angle = ang_img[grasp_point]

        g = Grasp(grasp_point, grasp_angle)
        if width_img is not None:
            g.length = width_img[grasp_point]
            g.width = g.length / 2

        grasps.append(g)

    return grasps
