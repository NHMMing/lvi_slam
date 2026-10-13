"""滤波一致性：NEES（归一化估计误差平方）。

  NEES = eᵀ P⁻¹ e，e 为真实误差，P 为滤波器给出的协方差。
  一致的滤波器 NEES/维度 ≈ 1：
    显著 > 1  过度自信——协方差比真实误差小，下游（后端因子权重、GPS 门槛、
              退化判定）都会被误导
    显著 < 1  过度保守——白白浪费了观测信息
  本系统宣称「权重全部由噪声模型推导」，NEES 是检验这一主张最直接的指标。

误差定义与 frames.csv 的协方差约定一致：
  e_rot = Log(R̂ᵀ R_true)   右扰动、body 系
  e_pos = p_true − p̂        世界系
真值先用开头一段做 4 自由度对齐，表达到估计的世界系里（滤波器的世界系锚定在首帧，
全程对齐会把误差摊平，不符合协方差的含义）。
"""
import math

import numpy as np

from .geometry import so3_log


def chi2_cdf(x, k):
    """卡方分布 CDF = 正则化下不完全 Gamma P(k/2, x/2)，级数展开（x 不大时收敛很快）。"""
    if x <= 0:
        return 0.0
    a, z = 0.5 * k, 0.5 * x
    term = total = 1.0 / a
    n = 0
    while abs(term) > 1e-15 * abs(total) and n < 10000:
        n += 1
        term *= z / (a + n)
        total += term
    return float(np.exp(-z + a * np.log(z) - math.lgamma(a)) * total)


def chi2_ppf(p, k):
    """卡方分位数：对 CDF 二分求逆。省掉 scipy 依赖。"""
    lo, hi = 0.0, max(10.0 * k, 50.0)
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if chi2_cdf(mid, k) < p:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def covariances(frames):
    """frames.csv 的 21 个上三角元素 → (N,6,6)。"""
    n = len(frames["t"])
    P = np.zeros((n, 6, 6))
    for i in range(6):
        for j in range(i, 6):
            v = frames["cov_%d%d" % (i, j)]
            P[:, i, j] = v
            P[:, j, i] = v
    return P


def nees(est, gt_in_est_frame, frames):
    """est 为 IMU 系逐帧位姿，gt_in_est_frame 是已关联并变换到估计世界系、IMU 系的真值。
    frames 为 frames.csv 列字典。按时间戳把协方差对上。"""
    if frames is None or "cov_00" not in frames:
        return None, None
    ft = frames["t"]
    ok_rows = frames["ok"] > 0
    ft, rows = ft[ok_rows], np.nonzero(ok_rows)[0]
    idx = np.searchsorted(ft, est.t)
    idx = np.clip(idx, 0, len(ft) - 1)
    match = np.abs(ft[idx] - est.t) < 1e-6
    if match.sum() < 10:
        return None, None
    P = covariances(frames)[rows[idx[match]]]
    e_pos = gt_in_est_frame.p[match] - est.p[match]
    out = {"count": int(match.sum())}
    series = {"t": est.t[match]}

    def one(name, e, Pm, k):
        try:
            v = np.einsum("ni,ni->n", e, np.linalg.solve(Pm, e[..., None])[..., 0])
        except np.linalg.LinAlgError:
            return
        lo, hi = chi2_ppf(0.025, k), chi2_ppf(0.975, k)
        out[name] = {"dof": k, "mean_nees": float(np.mean(v)),
                     "mean_nees_per_dof": float(np.mean(v) / k),
                     "median_nees_per_dof": float(np.median(v) / k),
                     "pct_in_95": float(100.0 * np.mean((v >= lo) & (v <= hi))),
                     "pct_above_95": float(100.0 * np.mean(v > hi)),
                     "bound_lo": float(lo), "bound_hi": float(hi)}
        series[name] = v

    one("pos", e_pos, P[:, 3:, 3:], 3)
    if gt_in_est_frame.has_rot:
        e_rot = so3_log(np.swapaxes(est.R[match], 1, 2) @ gt_in_est_frame.R[match])
        one("rot", e_rot, P[:, :3, :3], 3)
        one("pose", np.hstack([e_rot, e_pos]), P, 6)
    # 结论
    ref = out.get("pose") or out.get("pos")
    if ref:
        r = ref["mean_nees_per_dof"]
        out["verdict"] = ("过度自信（协方差偏小）" if r > 2 else
                          "过度保守（协方差偏大）" if r < 0.5 else "基本一致")
    return out, series
