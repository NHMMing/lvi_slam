"""测试用合成数据：解析轨迹 + 可控误差，以及伪造的 lvi_offline 运行目录。"""
import json
import os

import numpy as np

from lvi_eval.geometry import Trajectory, rot_z, so3_exp
from lvi_eval.io import write_tum


def figure_eight(duration=120.0, rate=10.0, radius=20.0, z_amp=1.0):
    """水平 8 字 + 轻微起伏，姿态朝向切线方向。回到起点，适合测回环。"""
    t = np.arange(0.0, duration, 1.0 / rate)
    w = 2 * np.pi / duration
    x = radius * np.sin(w * t)
    y = radius * np.sin(w * t) * np.cos(w * t)
    z = z_amp * np.sin(3 * w * t)
    p = np.stack([x, y, z], axis=1)
    v = np.gradient(p, t, axis=0)
    yaw = np.arctan2(v[:, 1], v[:, 0])
    R = np.stack([rot_z(a) for a in yaw])
    return Trajectory(t + 1000.0, p, R)


def with_drift(gt, trans_pct=1.0, yaw_rate_deg_per_100m=0.0, seed=0):
    """沿路程按比例累积平移误差（尺度漂移）+ 按路程累积的 yaw 漂移。"""
    d = gt.distances()
    s = 1.0 + trans_pct / 100.0
    # 尺度漂移：位移按 s 放大（相对起点）
    p = gt.p[0] + s * (gt.p - gt.p[0])
    R = gt.R.copy()
    if yaw_rate_deg_per_100m:
        yaw = np.radians(yaw_rate_deg_per_100m) * d / 100.0
        Rz = np.stack([rot_z(a) for a in yaw])
        # 绕起点旋转位置，同时旋转姿态
        p = gt.p[0] + np.einsum("nij,nj->ni", Rz, p - gt.p[0])
        R = Rz @ R
    return Trajectory(gt.t, p, R)


def with_noise(gt, sigma_pos, sigma_rot, seed=0):
    rng = np.random.RandomState(seed)
    dp = rng.randn(len(gt), 3) * sigma_pos
    dr = rng.randn(len(gt), 3) * sigma_rot
    return Trajectory(gt.t, gt.p + dp, gt.R @ so3_exp(dr)), dp, dr


def fake_run(path, est, slam=None, keyframes=None, frames_cov=None, loops=None, meta=None):
    """写一个与 lvi_offline --out-dir 格式一致的运行目录。frames_cov: (N,6,6) 或 None。"""
    os.makedirs(path, exist_ok=True)
    write_tum(os.path.join(path, "odometry.tum"), est)
    if slam is not None:
        write_tum(os.path.join(path, "slam.tum"), slam)
    if keyframes is not None:
        write_tum(os.path.join(path, "keyframes.tum"), keyframes)
    n = len(est)
    cols = ["t", "mode", "ok", "proc_ms", "iterations", "num_valid", "mean_error", "converged",
            "degenerate_mask", "vio_track", "td", "bg_x", "bg_y", "bg_z", "ba_x", "ba_y", "ba_z"]
    cols += ["cov_%d%d" % (i, j) for i in range(6) for j in range(i, 6)]
    P = frames_cov if frames_cov is not None else np.tile(np.eye(6) * 1e-4, (n, 1, 1))
    rng = np.random.RandomState(1)
    with open(os.path.join(path, "frames.csv"), "w") as f:
        f.write(",".join(cols) + "\n")
        for i in range(n):
            row = [est.t[i], 1, 1, 20 + 5 * rng.rand(), 3, 1500, 0.01, 1,
                   4 if 300 <= i < 320 else 0, -1, 0.001, 0, 0, 0, 0, 0, 0]
            row += [P[i, a, b] for a in range(6) for b in range(a, 6)]
            f.write(",".join("%.9f" % v if isinstance(v, float) else str(v) for v in row) + "\n")
    with open(os.path.join(path, "loops.csv"), "w") as f:
        f.write("stage,id_from,id_to,t_from,t_to,sc_dist,icp_fitness,dx,dy,dz,qx,qy,qz,qw\n")
        for lp in loops or []:
            f.write(",".join(str(x) for x in lp) + "\n")
    with open(os.path.join(path, "resources.csv"), "w") as f:
        f.write("wall_s,data_s,rss_mb,cpu_pct\n")
        for k in range(40):
            f.write("%f,%f,%f,%f\n" % (0.5 * k, 3.0 * k, 300 + min(k, 20) * 5, 150.0))
    m = {"git_rev": "test", "build_type": "Release", "data_start": float(est.t[0] - 0.5),
         "data_span_s": float(est.t[-1] - est.t[0] + 1.0), "wall_s": 20.0, "realtime_factor": 6.0,
         "peak_rss_mb": 400.0, "dropped": {"imu": 0, "cloud": 0, "image": 0}}
    m.update(meta or {})
    with open(os.path.join(path, "run_meta.json"), "w") as f:
        json.dump(m, f)
    return path
