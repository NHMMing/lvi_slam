"""精度指标：时间关联、对齐、ATE、RPE。

和 evo 的主要差别（都是为了让数字更「诚实」）：
  * 默认 4 自由度对齐（yaw + xyz），不是 6 自由度——惯性系统 roll/pitch 可观，
    不该被对齐掉
  * 除全程对齐的 ATE 外，还给出「只用开头一段对齐」的漂移误差，
    反映的是在线使用时真正会累积的误差
  * 水平 / 竖直误差分开
  * 支持只有位置的真值和杆臂外参
"""
import numpy as np

from .geometry import (Trajectory, align_pos_yaw, rot_angle, slerp, umeyama)
from .io import EvalError


# ---------------------------------------------------------------------------
# 时间关联
# ---------------------------------------------------------------------------
def interpolate(traj, t_query, max_gap):
    """在 t_query 处插值 traj（位置线性、姿态 slerp）。
    两侧最近样本间隔超过 max_gap 的查询点视为无真值。返回 (mask, Trajectory)。"""
    t = traj.t
    n = len(t)
    t_query = np.asarray(t_query, dtype=float)
    idx = np.searchsorted(t, t_query)            # t[idx-1] < tq <= t[idx]
    i1 = np.clip(idx, 0, n - 1)
    i0 = np.clip(idx - 1, 0, n - 1)
    exact = np.abs(t[i1] - t_query) < 1e-9
    inside = (idx > 0) & (idx < n)
    valid = exact | (inside & (t[i1] - t[i0] <= max_gap))
    i0 = np.where(exact, i1, i0)
    dt = t[i1] - t[i0]
    alpha = np.where(dt > 0, (t_query - t[i0]) / np.where(dt > 0, dt, 1.0), 0.0)
    p = traj.p[i0] + alpha[:, None] * (traj.p[i1] - traj.p[i0])
    R = slerp(traj.R[i0], traj.R[i1], alpha) if traj.has_rot else None
    return valid, Trajectory(t_query, p, R).subset(valid)


def associate(est, gt, max_gap=0.5):
    """把真值插值到估计的时间戳上。返回 (mask, est_matched, gt_matched)，mask 对应 est 的下标。"""
    if len(est) == 0:
        raise EvalError("估计轨迹为空")
    lo, hi = max(est.t[0], gt.t[0]), min(est.t[-1], gt.t[-1])
    if hi <= lo:
        raise EvalError(
            "估计与真值时间不重叠：估计 [%.3f, %.3f]，真值 [%.3f, %.3f]。\n"
            "  常见原因：雷达 header.stamp 用的是开机时钟（几百秒）而真值是 UNIX 时间，"
            "用 --t-offset 补上两者之差。" % (est.t[0], est.t[-1], gt.t[0], gt.t[-1]))
    mask, gt_m = interpolate(gt, est.t, max_gap)
    if mask.sum() < 10:
        raise EvalError("只有 %d 个时间戳能匹配到真值（--max-gap=%.2fs 太小？）" % (mask.sum(), max_gap))
    return mask, est.subset(mask), gt_m


# ---------------------------------------------------------------------------
# 对齐
# ---------------------------------------------------------------------------
ALIGN_MODES = ("posyaw", "se3", "sim3", "none")


def compute_alignment(est, gt, mode, idx=None):
    """求把 est 对齐到 gt 的 (R, t, s)；idx 指定只用部分点（开头一段）。"""
    src = est.p if idx is None else est.p[idx]
    dst = gt.p if idx is None else gt.p[idx]
    if mode == "none":
        return np.eye(3), np.zeros(3), 1.0
    if mode == "posyaw":
        return align_pos_yaw(src, dst)
    if mode == "se3":
        return umeyama(src, dst, with_scale=False)
    if mode == "sim3":
        return umeyama(src, dst, with_scale=True)
    raise EvalError("未知对齐方式 %s（%s）" % (mode, " | ".join(ALIGN_MODES)))


def head_indices(gt, head_m):
    """轨迹开头 head_m 米对应的样本下标。路程不够时退化为前 10% 的路程。"""
    d = gt.distances()
    total = d[-1] if len(d) else 0.0
    limit = head_m if total > 2 * head_m else 0.1 * total
    idx = np.nonzero(d <= max(limit, 1e-9))[0]
    if len(idx) < 3:
        idx = np.arange(min(3, len(d)))
    return idx


# ---------------------------------------------------------------------------
# ATE
# ---------------------------------------------------------------------------
def _stats(x):
    x = np.asarray(x, dtype=float)
    if len(x) == 0:
        return {}
    return {"rmse": float(np.sqrt(np.mean(x ** 2))), "mean": float(np.mean(x)),
            "median": float(np.median(x)), "std": float(np.std(x)),
            "max": float(np.max(x)), "p95": float(np.percentile(x, 95))}


def ate(est, gt, align="posyaw", head_idx=None):
    """est、gt 已逐点关联。返回 (指标字典, 逐点误差字典)。"""
    R, t, s = compute_alignment(est, gt, align, head_idx)
    est_a = est.transformed(R, t, s)
    e = est_a.p - gt.p
    trans = np.linalg.norm(e, axis=1)
    res = {"trans": _stats(trans),
           "horizontal": _stats(np.linalg.norm(e[:, :2], axis=1)),
           "vertical": _stats(np.abs(e[:, 2])),
           "final_trans": float(trans[-1]),
           "scale": s}
    series = {"t": est.t, "trans": trans, "horizontal": np.linalg.norm(e[:, :2], axis=1),
              "vertical": e[:, 2], "aligned": est_a}
    if gt.has_rot:
        rot = np.degrees(rot_angle(np.swapaxes(gt.R, 1, 2) @ est_a.R))
        res["rot_deg"] = _stats(rot)
        series["rot_deg"] = rot
    return res, series


# ---------------------------------------------------------------------------
# RPE
# ---------------------------------------------------------------------------
def default_lengths(path_len):
    """按路程挑段长：KITTI 的 100..800m 只适合车载；手持/室内要更短的段。"""
    cands = [5, 10, 20, 50, 100, 200, 400, 800]
    ls = [L for L in cands if L <= 0.5 * path_len]
    return ls[-6:] if ls else [max(1.0, round(0.25 * path_len, 1))]


def _relative_errors(est, gt, i, j):
    """段 i→j 的相对位姿误差。完整真值用 SE(3) 相对误差；只有位置时比较世界系位移
    （est 需已对齐）。返回 (平移误差 m, 旋转误差 deg 或 None)。"""
    if gt.has_rot:
        Rg = np.swapaxes(gt.R[i], 1, 2) @ gt.R[j]
        tg = np.einsum("nji,nj->ni", gt.R[i], gt.p[j] - gt.p[i])
        Re = np.swapaxes(est.R[i], 1, 2) @ est.R[j]
        te = np.einsum("nji,nj->ni", est.R[i], est.p[j] - est.p[i])
        # E = (T_gt_ij)⁻¹ · T_est_ij
        RE = np.swapaxes(Rg, 1, 2) @ Re
        tE = np.einsum("nji,nj->ni", Rg, te - tg)
        return np.linalg.norm(tE, axis=1), np.degrees(rot_angle(RE))
    de = est.p[j] - est.p[i]
    dg = gt.p[j] - gt.p[i]
    return np.linalg.norm(de - dg, axis=1), None


def rpe_distance(est, gt, lengths, est_aligned=None):
    """按路程分段的漂移率（KITTI 口径）：平移 %、旋转 deg/100m。"""
    d = gt.distances()
    use = est if gt.has_rot else est_aligned
    out = {"segments": []}
    all_t, all_r = [], []
    for L in lengths:
        step = max(1.0, L / 20.0)   # 起点间隔，控制样本数
        starts, next_d = [], 0.0
        for i, di in enumerate(d):
            if di >= next_d:
                starts.append(i)
                next_d = di + step
        starts = np.array(starts, dtype=int)
        ends = np.searchsorted(d, d[starts] + L)
        ok = ends < len(d)
        i, j = starts[ok], ends[ok]
        if len(i) == 0:
            continue
        te, re = _relative_errors(use, gt, i, j)
        seg_len = d[j] - d[i]
        tp = 100.0 * te / seg_len
        rec = {"length_m": L, "count": int(len(i)),
               "trans_pct": float(np.mean(tp)), "trans_pct_median": float(np.median(tp)),
               "trans_pct_max": float(np.max(tp))}
        all_t.append(tp)
        if re is not None:
            rr = 100.0 * re / seg_len
            rec["rot_deg_per_100m"] = float(np.mean(rr))
            all_r.append(rr)
        out["segments"].append(rec)
    if all_t:
        out["trans_pct"] = float(np.mean(np.concatenate(all_t)))
    if all_r:
        out["rot_deg_per_100m"] = float(np.mean(np.concatenate(all_r)))
    return out


def rpe_time(est, gt, delta, est_aligned=None):
    """固定时间间隔的相对误差，反映局部平滑度 / 抖动。"""
    use = est if gt.has_rot else est_aligned
    j = np.searchsorted(gt.t, gt.t + delta)
    ok = j < len(gt.t)
    i = np.nonzero(ok)[0]
    j = j[ok]
    if len(i) == 0:
        return {}, None
    # 时间戳不严格等于 delta 时（帧率抖动），只保留接近的
    good = np.abs((gt.t[j] - gt.t[i]) - delta) < 0.5 * delta
    i, j = i[good], j[good]
    if len(i) == 0:
        return {}, None
    te, re = _relative_errors(use, gt, i, j)
    res = {"delta_s": delta, "count": int(len(i)), "trans": _stats(te)}
    if re is not None:
        res["rot_deg"] = _stats(re)
    return res, (gt.t[i], te)
