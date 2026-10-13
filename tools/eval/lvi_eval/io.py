"""读取真值、估计轨迹和 lvi_offline --out-dir 产出的运行目录。"""
import csv
import json
import os

import numpy as np

from .geometry import Trajectory, quat_to_rot


class EvalError(Exception):
    """用户可修正的输入问题（文件缺失、格式不对、时间不重叠）。CLI 只打印消息不打栈。"""


# ---------------------------------------------------------------------------
# 轨迹文件
# ---------------------------------------------------------------------------
def _fix_time_unit(t):
    """纳秒时间戳（EuRoC、部分 RTK 导出）自动换成秒。"""
    if len(t) and np.median(np.abs(t)) > 1e14:
        return t * 1e-9
    return t


def _load_numeric(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line[0] in "#%":
                continue
            parts = line.replace(",", " ").split()
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                continue   # 表头
    if not rows:
        raise EvalError("%s: 没有可读的数据行" % path)
    width = min(len(r) for r in rows)
    return np.array([r[:width] for r in rows])


def load_tum(path):
    """TUM: t x y z qx qy qz qw；只有 4 列 (t x y z) 时视为只有位置的真值。"""
    a = _load_numeric(path)
    t = _fix_time_unit(a[:, 0])
    if a.shape[1] >= 8:
        return Trajectory(t, a[:, 1:4], quat_to_rot(a[:, 4:8])).sorted_unique()
    if a.shape[1] >= 4:
        return Trajectory(t, a[:, 1:4]).sorted_unique()
    raise EvalError("%s: 列数 %d，TUM 需要 4（仅位置）或 8 列" % (path, a.shape[1]))


def load_euroc(path):
    """EuRoC/ASL csv: t[ns], px, py, pz, qw, qx, qy, qz, ...（四元数 w 在前）。"""
    a = _load_numeric(path)
    if a.shape[1] < 8:
        raise EvalError("%s: EuRoC 格式至少 8 列" % path)
    q_xyzw = a[:, [5, 6, 7, 4]]
    return Trajectory(_fix_time_unit(a[:, 0]), a[:, 1:4], quat_to_rot(q_xyzw)).sorted_unique()


def load_trajectory(path, fmt="auto"):
    if not os.path.isfile(path):
        raise EvalError("找不到轨迹文件: %s" % path)
    if fmt == "auto":
        fmt = "tum"
        if path.endswith(".csv"):
            with open(path) as f:
                head = f.readline().lower()
            if "q_rs_w" in head or "qw" in head.replace(" ", "") or "#timestamp" in head:
                fmt = "euroc"
    if fmt == "tum":
        return load_tum(path)
    if fmt == "euroc":
        return load_euroc(path)
    raise EvalError("未知轨迹格式 %s（tum | euroc）" % fmt)


def write_tum(path, traj):
    from .geometry import rot_to_quat
    with open(path, "w") as f:
        q = rot_to_quat(traj.R) if traj.has_rot else None
        for i in range(len(traj)):
            row = [traj.t[i]] + list(traj.p[i])
            if q is not None:
                row += list(q[i])
            f.write(" ".join("%.9f" % v for v in row) + "\n")


def parse_extrinsic(text):
    """'x y z' 或 'x y z qx qy qz qw'（逗号或空格分隔）→ (R, t)。"""
    if text is None:
        return None
    v = [float(x) for x in text.replace(",", " ").split()]
    if len(v) == 3:
        return np.eye(3), np.array(v)
    if len(v) == 7:
        return quat_to_rot(np.array(v[3:]))[...], np.array(v[:3])
    raise EvalError("外参需要 3 个（杆臂）或 7 个（平移+四元数 xyzw）数: %r" % text)


# ---------------------------------------------------------------------------
# CSV → 列字典
# ---------------------------------------------------------------------------
def read_csv_columns(path):
    """返回 {列名: np.ndarray}；数值列转 float，非数值列保留字符串数组。"""
    with open(path, newline="") as f:
        reader = csv.reader(f)
        try:
            header = next(reader)
        except StopIteration:
            return {}
        rows = [r for r in reader if r]
    cols = {}
    for k, name in enumerate(header):
        raw = [r[k] if k < len(r) else "" for r in rows]
        try:
            cols[name] = np.array([float(x) for x in raw])
        except ValueError:
            cols[name] = np.array(raw)
    return cols


# ---------------------------------------------------------------------------
# 运行目录
# ---------------------------------------------------------------------------
class Run:
    """lvi_offline --out-dir 的产物。缺失的文件对应属性为 None，各项指标按需降级。"""

    def __init__(self, path):
        if not os.path.isdir(path):
            raise EvalError("运行目录不存在: %s" % path)
        self.path = path
        self.name = os.path.basename(os.path.normpath(path))

        def opt(fn, loader):
            fp = os.path.join(path, fn)
            if not os.path.isfile(fp) or os.path.getsize(fp) == 0:
                return None
            try:
                return loader(fp)
            except EvalError:
                return None

        self.odometry = opt("odometry.tum", load_tum)
        self.slam = opt("slam.tum", load_tum)
        self.keyframes = opt("keyframes.tum", load_tum)
        self.frames = opt("frames.csv", read_csv_columns)
        self.loops = opt("loops.csv", read_csv_columns)
        self.resources = opt("resources.csv", read_csv_columns)
        meta_path = os.path.join(path, "run_meta.json")
        self.meta = {}
        if os.path.isfile(meta_path):
            with open(meta_path) as f:
                self.meta = json.load(f)
        if self.odometry is None:
            # 兼容旧用法：目录里只有 --out 产出的单条轨迹
            for fn in ("trajectory.tum",):
                fp = os.path.join(path, fn)
                if os.path.isfile(fp):
                    self.odometry = load_tum(fp)
        if self.meta.get("crashed"):
            raise EvalError("lvi_offline 异常退出（exit %s），见 %s/console.log"
                            % (self.meta.get("exit_code"), path))
        if self.odometry is None:
            raise EvalError("%s 里没有 odometry.tum，不是 lvi_offline --out-dir 的输出目录？" % path)

    def trajectories(self):
        """[(名字, 轨迹)]，按 odometry → slam → keyframes 的顺序，跳过缺失的。"""
        out = [("odometry", self.odometry)]
        if self.slam is not None and len(self.slam) > 0:
            out.append(("slam", self.slam))
        if self.keyframes is not None and len(self.keyframes) > 0:
            out.append(("keyframes", self.keyframes))
        return out


def is_run_dir(path):
    return os.path.isfile(os.path.join(path, "odometry.tum")) or \
        os.path.isfile(os.path.join(path, "run_meta.json"))
