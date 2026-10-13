"""单次运行的完整评估：把各模块的指标汇总成一个 summary 字典（同时保留画图用的序列）。"""
import numpy as np

from . import accuracy, consistency, loops, robustness, runtime
from .geometry import Trajectory


class Options:
    """评估参数。默认值面向 LiDAR-惯性系统；CLI 的同名参数会覆盖。"""

    def __init__(self, **kw):
        self.t_offset = 0.0           # 加到估计时间戳上
        self.extrinsic = None         # (R_bx, t_bx)：真值点在 IMU 系下的位姿
        self.max_gap = 0.5            # 真值插值允许的最大样本间隔 (s)
        self.align = "posyaw"         # 全程 ATE 的对齐方式
        self.head_m = 20.0            # 漂移误差：只用开头这么多米对齐
        self.rpe_lengths = None       # None = 按路程自动选
        self.rpe_deltas = [1.0, 10.0]
        self.jump_m = 0.3             # 单帧相对运动与真值偏差超过即算跳变
        self.jump_deg = 3.0
        self.max_acc = 30.0           # 无真值时的跳变加速度门槛 (m/s²)
        self.fail_abs = 5.0           # 发散：漂移误差 > max(fail_abs, fail_rel × 路程)
        self.fail_rel = 0.05
        self.min_coverage = 0.95      # 成功判据：输出覆盖率
        self.success_ate = None       # 成功判据：ATE RMSE 上限 (m)，None = 不限
        self.closed_loop = False      # 序列首尾重合（无真值时用端点漂移）
        self.loop_radius = 5.0
        self.loop_trans = 1.0
        self.loop_rot = 5.0
        self.loop_min_dt = 30.0
        for k, v in kw.items():
            if not hasattr(self, k):
                raise TypeError("未知选项 %s" % k)
            setattr(self, k, v)

    def as_dict(self):
        d = dict(self.__dict__)
        if d["extrinsic"] is not None:
            R, t = d["extrinsic"]
            d["extrinsic"] = {"R": np.asarray(R).tolist(), "t": np.asarray(t).tolist()}
        return d


def _inverse_offset(R_bx, t_bx):
    return R_bx.T, -R_bx.T @ t_bx


def evaluate_run(run, gt, opt):
    """返回 (summary, series)。gt 可为 None（只算无真值指标）。"""
    summary = {"run": run.path, "meta": run.meta, "options": opt.as_dict(), "trajectories": {}}
    series = {"trajectories": {}}
    reasons = []

    if gt is not None:
        summary["gt"] = {"poses": len(gt), "has_rot": gt.has_rot,
                         "duration_s": float(gt.t[-1] - gt.t[0]), "path_m": gt.length()}

    for name, traj in run.trajectories():
        est = traj.with_time_offset(opt.t_offset)
        rec = {"poses": len(est), "motion": robustness.motion_stats(est)}
        ser = {"est": est}
        est_x = est.with_body_offset(*opt.extrinsic) if opt.extrinsic is not None else est
        if gt is not None:
            mask, e_m, g_m = accuracy.associate(est_x, gt, opt.max_gap)
            rec["matched"] = int(mask.sum())
            ate, ate_s = accuracy.ate(e_m, g_m, opt.align)
            head = accuracy.head_indices(g_m, opt.head_m)
            drift, drift_s = accuracy.ate(e_m, g_m, "posyaw", head)
            rec["ate"], rec["ate_drift"] = ate, drift
            lengths = opt.rpe_lengths or accuracy.default_lengths(g_m.length())
            rec["rpe_distance"] = accuracy.rpe_distance(e_m, g_m, lengths, ate_s["aligned"])
            rec["rpe_time"] = {}
            for dlt in opt.rpe_deltas:
                r, s = accuracy.rpe_time(e_m, g_m, dlt, ate_s["aligned"])
                if r:
                    rec["rpe_time"]["%gs" % dlt] = r
                    ser["rpe_time_%gs" % dlt] = s
            ser.update({"mask": mask, "est_m": e_m, "gt_m": g_m, "ate": ate_s, "drift": drift_s})
        summary["trajectories"][name] = rec
        series["trajectories"][name] = ser

    odo = summary["trajectories"]["odometry"]
    odo_s = series["trajectories"]["odometry"]

    # ---- 稳定性 ----
    rob = {}
    gaps, gap_thr = robustness.output_gaps(odo_s["est"].t)
    rob["gaps"] = {"count": len(gaps), "threshold_s": gap_thr,
                   "total_s": float(sum(g[1] for g in gaps)), "list": gaps[:20]}
    data_start = run.meta.get("data_start")
    if data_start is not None and data_start > 0:
        rob["init_time_s"] = float(odo_s["est"].t[0] - opt.t_offset - data_start)
    if gt is not None:
        ref_start = max(gt.t[0], (data_start + opt.t_offset) if data_start else gt.t[0])
        ref_end = gt.t[-1]
        if run.meta.get("data_span_s") and data_start:
            ref_end = min(ref_end, data_start + opt.t_offset + run.meta["data_span_s"])
        rob["coverage"] = robustness.coverage(odo_s["est"].t, ref_start, ref_end, gap_thr)
        for name in ("odometry", "slam"):
            if name not in series["trajectories"] or "est_m" not in series["trajectories"][name]:
                continue
            s = series["trajectories"][name]
            j = robustness.jumps_vs_gt(s["est_m"], s["gt_m"], opt.jump_m, opt.jump_deg)
            rob["jumps_" + name] = {"count": len(j), "list": j[:20]}
            dv = robustness.divergence(s["drift"]["t"], s["drift"]["trans"],
                                       s["gt_m"].distances(), opt.fail_abs, opt.fail_rel)
            rob["divergence_" + name] = dv
    else:
        j = robustness.jumps_no_gt(odo_s["est"], opt.max_acc)
        rob["jumps_odometry"] = {"count": len(j), "list": j[:20]}
    if opt.closed_loop:
        for name, s in series["trajectories"].items():
            if name != "keyframes":
                rob["endpoint_" + name] = robustness.endpoint_drift(s["est"])
    summary["robustness"] = rob

    # ---- 后端收益 ----
    if "slam" in summary["trajectories"]:
        slam = summary["trajectories"]["slam"]
        if gt is not None and "ate" in slam:
            a, b = odo["ate"]["trans"]["rmse"], slam["ate"]["trans"]["rmse"]
            c, d = odo["ate_drift"]["trans"]["rmse"], slam["ate_drift"]["trans"]["rmse"]
            summary["backend_gain"] = {
                "ate_rmse_odometry": a, "ate_rmse_slam": b,
                "ate_improvement_pct": 100.0 * (a - b) / a if a > 0 else 0.0,
                "drift_rmse_odometry": c, "drift_rmse_slam": d,
                "drift_improvement_pct": 100.0 * (c - d) / c if c > 0 else 0.0}
        # 无论有没有真值，都给出后端把轨迹改动了多少（odometry 相对 slam 的偏差）
        try:
            _, e_m, g_m = accuracy.associate(odo_s["est"], series["trajectories"]["slam"]["est"], 0.05)
            diff = np.linalg.norm(e_m.p - g_m.p, axis=1)
            summary.setdefault("backend_gain", {})["odometry_vs_slam_max_m"] = float(np.max(diff))
            summary["backend_gain"]["odometry_vs_slam_rmse_m"] = float(np.sqrt(np.mean(diff ** 2)))
        except Exception:
            pass

    # ---- 一致性（NEES）----
    if gt is not None and run.frames is not None and "est_m" in odo_s:
        est_imu = odo_s["est"].subset(odo_s["mask"])
        g = odo_s["gt_m"]
        # 真值 → 估计世界系：用漂移对齐（开头一段）的逆变换
        R_a, t_a, _ = accuracy.compute_alignment(
            odo_s["est_m"], g, "posyaw", accuracy.head_indices(g, opt.head_m))
        g_w = Trajectory(g.t, (g.p - t_a) @ R_a, None if not g.has_rot else R_a.T @ g.R)
        # 真值点 → IMU 原点
        if opt.extrinsic is not None:
            R_bx, t_bx = opt.extrinsic
            if g_w.has_rot:
                g_w = g_w.with_body_offset(*_inverse_offset(R_bx, t_bx))
            else:
                p = g_w.p - np.einsum("nij,j->ni", est_imu.R, t_bx)
                g_w = Trajectory(g_w.t, p)
        nees, nees_s = consistency.nees(est_imu, g_w, run.frames)
        if nees:
            summary["consistency"] = nees
            series["nees"] = nees_s

    # ---- 回环 ----
    if run.loops is not None or gt is not None:
        gt_imu = gt
        if gt is not None and opt.extrinsic is not None and gt.has_rot:
            gt_imu = gt.with_body_offset(*_inverse_offset(*opt.extrinsic))
        lp, lp_d = loops.evaluate(run.loops, run.keyframes, gt_imu, opt.loop_radius,
                                  opt.loop_trans, opt.loop_rot, opt.loop_min_dt, opt.max_gap)
        summary["loops"] = lp
        series["loops"] = lp_d

    summary["efficiency"] = runtime.efficiency(run)
    summary["health"] = runtime.health(run)

    # ---- 结论 ----
    if gt is not None:
        cov = rob.get("coverage", 1.0)
        if cov < opt.min_coverage:
            reasons.append("输出覆盖率 %.1f%% < %.0f%%" % (100 * cov, 100 * opt.min_coverage))
        if rob.get("divergence_odometry"):
            dv = rob["divergence_odometry"]
            reasons.append("里程计在 %.1fs（%.0fm）处发散：误差 %.2fm > %.2fm"
                           % (dv["t_rel"], dv["dist_m"], dv["err_m"], dv["limit_m"]))
        best = summary["trajectories"].get("slam", odo).get("ate", odo["ate"])
        if opt.success_ate is not None and best["trans"]["rmse"] > opt.success_ate:
            reasons.append("ATE %.3fm > %.3fm" % (best["trans"]["rmse"], opt.success_ate))
    drops = run.meta.get("dropped", {})
    if sum(drops.values()) if drops else 0:
        reasons.append("缓冲溢出丢帧 %s" % drops)
    summary["verdict"] = {"success": not reasons, "reasons": reasons}
    return summary, series
