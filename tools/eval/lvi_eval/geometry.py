"""SO(3)/SE(3) 基础运算与轨迹容器。只依赖 numpy，所有函数都对批量 (N,3,3) 向量化。

约定：
  四元数 (x, y, z, w)，与 TUM 格式一致
  轨迹位姿 T_w_b：p 是 body 原点在世界系的位置，R 把 body 系向量转到世界系
"""
import numpy as np


# ---------------------------------------------------------------------------
# 旋转
# ---------------------------------------------------------------------------
def quat_to_rot(q):
    """(N,4) xyzw → (N,3,3)。输入不要求单位化。"""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = 1 - 2 * (y * y + z * z)
    R[..., 0, 1] = 2 * (x * y - z * w)
    R[..., 0, 2] = 2 * (x * z + y * w)
    R[..., 1, 0] = 2 * (x * y + z * w)
    R[..., 1, 1] = 1 - 2 * (x * x + z * z)
    R[..., 1, 2] = 2 * (y * z - x * w)
    R[..., 2, 0] = 2 * (x * z - y * w)
    R[..., 2, 1] = 2 * (y * z + x * w)
    R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def rot_to_quat(R):
    """(N,3,3) → (N,4) xyzw，w >= 0。"""
    R = np.asarray(R, dtype=float)
    single = R.ndim == 2
    R = R.reshape(-1, 3, 3)
    q = np.empty((R.shape[0], 4))
    tr = np.trace(R, axis1=1, axis2=2)
    for i in range(R.shape[0]):
        m = R[i]
        if tr[i] > 0:
            s = 2.0 * np.sqrt(tr[i] + 1.0)
            q[i] = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                    (m[1, 0] - m[0, 1]) / s, 0.25 * s]
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            q[i] = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
                    (m[2, 1] - m[1, 2]) / s]
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            q[i] = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s,
                    (m[0, 2] - m[2, 0]) / s]
        else:
            s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            q[i] = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s,
                    (m[1, 0] - m[0, 1]) / s]
    q[q[:, 3] < 0] *= -1
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return q[0] if single else q


def hat(v):
    v = np.asarray(v, dtype=float)
    M = np.zeros(v.shape[:-1] + (3, 3))
    M[..., 0, 1], M[..., 0, 2] = -v[..., 2], v[..., 1]
    M[..., 1, 0], M[..., 1, 2] = v[..., 2], -v[..., 0]
    M[..., 2, 0], M[..., 2, 1] = -v[..., 1], v[..., 0]
    return M


def so3_exp(phi):
    """(N,3) → (N,3,3)，Rodrigues。"""
    phi = np.asarray(phi, dtype=float)
    theta = np.linalg.norm(phi, axis=-1)[..., None, None]
    K = hat(phi)
    small = theta < 1e-8
    th = np.where(small, 1.0, theta)
    a = np.where(small, 1.0, np.sin(th) / th)
    b = np.where(small, 0.5, (1 - np.cos(th)) / (th * th))
    I = np.broadcast_to(np.eye(3), K.shape)
    return I + a * K + b * (K @ K)


def so3_log(R):
    """(N,3,3) → (N,3)。角度用 atan2 求（arccos 在 0 和 π 附近都丢精度），
    θ 接近 π 时 Rᵀ−R 退化，改由对称部分取转轴。"""
    R = np.asarray(R, dtype=float)
    single = R.ndim == 2
    R = R.reshape(-1, 3, 3)
    w = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0],
                  R[:, 1, 0] - R[:, 0, 1]], axis=1)
    sin2 = np.linalg.norm(w, axis=1)                       # 2·sinθ
    cos = (np.trace(R, axis1=1, axis2=2) - 1) / 2
    theta = np.arctan2(0.5 * sin2, cos)
    out = np.empty((R.shape[0], 3))
    small = theta < 1e-6
    near_pi = theta > np.pi - 1e-2
    reg = ~(small | near_pi)
    out[small] = 0.5 * w[small]
    out[reg] = (theta[reg] / sin2[reg])[:, None] * w[reg]
    for i in np.nonzero(near_pi)[0]:
        # (R+Rᵀ)/2 − cosθ·I = (1−cosθ)·aaᵀ：取对角最大的列作转轴，符号由反对称部分定
        S = 0.5 * (R[i] + R[i].T) - cos[i] * np.eye(3)
        k = int(np.argmax(np.diag(S)))
        axis = S[:, k] / np.linalg.norm(S[:, k])
        if np.dot(axis, w[i]) < 0:
            axis = -axis
        out[i] = theta[i] * axis
    return out[0] if single else out


def rot_angle(R):
    """(N,3,3) → (N,) 旋转角（弧度），atan2 形式，小角度下精确。"""
    R = np.asarray(R, dtype=float)
    w = np.stack([R[..., 2, 1] - R[..., 1, 2], R[..., 0, 2] - R[..., 2, 0],
                  R[..., 1, 0] - R[..., 0, 1]], axis=-1)
    cos = (np.trace(R, axis1=-2, axis2=-1) - 1) / 2
    return np.arctan2(0.5 * np.linalg.norm(w, axis=-1), cos)


def rot_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def slerp(R0, R1, alpha):
    """逐元素插值：R0·Exp(α·Log(R0ᵀR1))。"""
    d = so3_log(np.swapaxes(R0, -1, -2) @ R1)
    return R0 @ so3_exp(d * np.asarray(alpha)[..., None])


# ---------------------------------------------------------------------------
# 轨迹
# ---------------------------------------------------------------------------
class Trajectory:
    """时间戳 + 位置 + （可选）姿态。R 为 None 表示只有位置（RTK、全站仪棱镜）。"""

    def __init__(self, t, p, R=None):
        self.t = np.asarray(t, dtype=float)
        self.p = np.asarray(p, dtype=float).reshape(-1, 3)
        self.R = None if R is None else np.asarray(R, dtype=float).reshape(-1, 3, 3)
        assert len(self.t) == len(self.p)
        assert self.R is None or len(self.R) == len(self.t)

    @property
    def has_rot(self):
        return self.R is not None

    def __len__(self):
        return len(self.t)

    def subset(self, idx):
        return Trajectory(self.t[idx], self.p[idx], None if self.R is None else self.R[idx])

    def sorted_unique(self):
        """按时间排序并去掉重复时间戳（保留第一个）。"""
        order = np.argsort(self.t, kind="stable")
        tr = self.subset(order)
        keep = np.concatenate([[True], np.diff(tr.t) > 1e-9])
        return tr.subset(keep)

    def transformed(self, R, t, s=1.0):
        """左乘相似变换：p' = s·R·p + t，R' = R·Rᵢ。"""
        p = s * self.p @ np.asarray(R).T + np.asarray(t)
        Rs = None if self.R is None else np.asarray(R) @ self.R
        return Trajectory(self.t, p, Rs)

    def with_body_offset(self, R_bx, t_bx):
        """右乘体坐标系外参 T_b_x：把「body 系位姿」换成「刚连在 body 上的 x 系位姿」。
        真值测的往往不是 IMU 原点（GNSS 天线、全站仪棱镜），需要这一步才能比较。"""
        if self.R is None:
            raise ValueError("应用外参需要姿态")
        p = self.p + np.einsum("nij,j->ni", self.R, np.asarray(t_bx, dtype=float))
        return Trajectory(self.t, p, self.R @ np.asarray(R_bx, dtype=float))

    def with_time_offset(self, dt):
        return Trajectory(self.t + dt, self.p, self.R)

    def distances(self):
        """沿轨迹累计路程，(N,)。"""
        if len(self) == 0:
            return np.zeros(0)
        step = np.linalg.norm(np.diff(self.p, axis=0), axis=1)
        return np.concatenate([[0.0], np.cumsum(step)])

    def length(self):
        d = self.distances()
        return float(d[-1]) if len(d) else 0.0


# ---------------------------------------------------------------------------
# 对齐：求把 src 对齐到 dst 的变换 (R, t, s)，即 dst ≈ s·R·src + t
# ---------------------------------------------------------------------------
def umeyama(src, dst, with_scale=False):
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if len(src) < 3:
        raise ValueError("SE(3)/Sim(3) 对齐至少需要 3 个点")
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_s = (xs ** 2).sum() / len(src)
    s = float(np.trace(np.diag(D) @ S) / var_s) if with_scale else 1.0
    t = mu_d - s * R @ mu_s
    return R, t, s


def align_pos_yaw(src, dst):
    """4 自由度对齐（偏航 + 平移）。

    惯性系统的 roll/pitch 由重力绝对可观，只有 yaw 和 xyz 是不可观的 gauge 自由度。
    用 6 自由度对齐会把真实的 roll/pitch 误差「对齐掉」，结果偏乐观；4 自由度才公平。
    前提：估计和真值的 z 轴都与重力对齐（ENU、或静止初始化后的 IMU 世界系）。
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if len(src) < 2:
        raise ValueError("4 自由度对齐至少需要 2 个点")
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    a = np.sum(xs[:, 0] * xd[:, 0] + xs[:, 1] * xd[:, 1])
    b = np.sum(xs[:, 0] * xd[:, 1] - xs[:, 1] * xd[:, 0])
    R = rot_z(np.arctan2(b, a))
    return R, mu_d - R @ mu_s, 1.0
