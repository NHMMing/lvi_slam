"""稳定性指标：输出覆盖率、初始化时间、中断、跳变、发散判定、闭环端点漂移。

「平均误差小」不等于「稳定」：一次 3 秒的发散再自己拉回来，ATE RMSE 可能只涨一点，
但对下游控制是致命的。这里的指标专门把这类事件单独拎出来计数。
"""
import numpy as np

from .geometry import rot_angle


def output_gaps(t, gap_factor=5.0, min_gap=0.5):
    """输出中断：相邻位姿间隔超过 max(gap_factor × 中位周期, min_gap)。"""
    if len(t) < 2:
        return [], 0.0
    dt = np.diff(t)
    thresh = max(gap_factor * float(np.median(dt)), min_gap)
    idx = np.nonzero(dt > thresh)[0]
    gaps = [(float(t[i]), float(dt[i])) for i in idx]
    return gaps, thresh


def coverage(est_t, ref_start, ref_end, gap_thresh):
    """[ref_start, ref_end] 内有连续位姿输出的时间占比。"""
    span = ref_end - ref_start
    if span <= 0 or len(est_t) < 2:
        return 0.0
    t = est_t[(est_t >= ref_start) & (est_t <= ref_end)]
    if len(t) < 2:
        return 0.0
    dt = np.diff(t)
    covered = float(np.sum(dt[dt <= gap_thresh]))
    return min(1.0, covered / span)


def jumps_vs_gt(est, gt, jump_m, jump_deg):
    """逐帧相对运动与真值不一致的跳变（est、gt 已关联）。
    只对相邻且间隔正常的帧检查；位置用世界系位移差，避免受全局对齐影响。"""
    if len(est) < 3:
        return []
    dt = np.diff(est.t)
    normal = dt <= 3 * np.median(dt)
    de = np.diff(est.p, axis=0)
    dg = np.diff(gt.p, axis=0)
    # 位移在各自世界系里，长时间漂移会让 yaw 不一致：用 est 自身的姿态把位移转到 body 系比较
    if est.has_rot and gt.has_rot:
        de = np.einsum("nji,nj->ni", est.R[:-1], de)
        dg = np.einsum("nji,nj->ni", gt.R[:-1], dg)
        err_t = np.linalg.norm(de - dg, axis=1)
        dRe = np.swapaxes(est.R[:-1], 1, 2) @ est.R[1:]
        dRg = np.swapaxes(gt.R[:-1], 1, 2) @ gt.R[1:]
        err_r = np.degrees(rot_angle(np.swapaxes(dRg, 1, 2) @ dRe))
    else:
        err_t = np.abs(np.linalg.norm(de, axis=1) - np.linalg.norm(dg, axis=1))
        err_r = np.zeros_like(err_t)
    bad = normal & ((err_t > jump_m) | (err_r > jump_deg))
    return [{"t": float(est.t[i + 1]), "trans_m": float(err_t[i]), "rot_deg": float(err_r[i])}
            for i in np.nonzero(bad)[0]]


def jumps_no_gt(est, max_acc):
    """无真值时的跳变检测：二阶差分算出的加速度超过 max_acc（m/s²）。
    回环修正会在 odometry 上造成合法跳变，报告里会标注附近是否有回环。"""
    if len(est) < 3:
        return []
    t, p = est.t, est.p
    dt1, dt2 = np.diff(t)[:-1], np.diff(t)[1:]
    v1 = (p[1:-1] - p[:-2]) / dt1[:, None]
    v2 = (p[2:] - p[1:-1]) / dt2[:, None]
    acc = np.linalg.norm(v2 - v1, axis=1) / (0.5 * (dt1 + dt2))
    normal = (dt1 <= 3 * np.median(dt1)) & (dt2 <= 3 * np.median(dt2))
    idx = np.nonzero(normal & (acc > max_acc))[0]
    return [{"t": float(t[i + 1]), "acc": float(acc[i])} for i in idx]


def divergence(t, err, dist, abs_m, rel):
    """发散判定：漂移误差超过 max(abs_m, rel × 已行驶路程) 的第一个时刻。"""
    limit = np.maximum(abs_m, rel * dist)
    bad = np.nonzero(err > limit)[0]
    if len(bad) == 0:
        return None
    i = int(bad[0])
    return {"t": float(t[i]), "t_rel": float(t[i] - t[0]), "dist_m": float(dist[i]),
            "err_m": float(err[i]), "limit_m": float(limit[i])}


def endpoint_drift(est):
    """首尾位置差。只有序列确实回到起点时才有意义（R3LIVE / FAST-LIVO2 数据集）。"""
    L = est.length()
    d = float(np.linalg.norm(est.p[-1] - est.p[0]))
    return {"drift_m": d, "path_m": L, "drift_pct": 100.0 * d / L if L > 0 else None}


def motion_stats(est):
    if len(est) < 2:
        return {}
    dt = np.diff(est.t)
    ok = dt > 0
    v = np.linalg.norm(np.diff(est.p, axis=0), axis=1)[ok] / dt[ok]
    return {"duration_s": float(est.t[-1] - est.t[0]), "path_m": est.length(),
            "speed_mean": float(np.mean(v)), "speed_max": float(np.max(v)),
            "rate_hz": float(1.0 / np.median(dt))}
