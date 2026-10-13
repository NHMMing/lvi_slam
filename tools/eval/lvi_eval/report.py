"""输出：终端摘要、summary.json、report.md 和图。"""
import json
import os

import numpy as np

COLORS = {"gt": "#555555", "odometry": "#d1495b", "slam": "#2e86ab", "keyframes": "#66a182"}


def _fmt(v, nd=3, unit=""):
    if v is None:
        return "—"
    if isinstance(v, (int, np.integer)):
        return "%d%s" % (v, unit)
    return ("%." + str(nd) + "f%s") % (v, unit)


def _get(d, *keys):
    for k in keys:
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d


def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.bool_):
        return bool(o)
    return str(o)


def write_json(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False, default=_json_default)


# ---------------------------------------------------------------------------
# 终端 / Markdown
# ---------------------------------------------------------------------------
def accuracy_table(summary):
    rows = [["轨迹", "匹配位姿", "ATE RMSE (m)", "ATE max", "水平 RMSE", "竖直 RMSE",
             "旋转 RMSE (°)", "漂移 RMSE (m)", "RPE 平移 %", "RPE °/100m"]]
    for name, r in summary["trajectories"].items():
        if "ate" not in r:
            continue
        rows.append([name, str(r["matched"]),
                     _fmt(_get(r, "ate", "trans", "rmse")), _fmt(_get(r, "ate", "trans", "max")),
                     _fmt(_get(r, "ate", "horizontal", "rmse")), _fmt(_get(r, "ate", "vertical", "rmse")),
                     _fmt(_get(r, "ate", "rot_deg", "rmse"), 2),
                     _fmt(_get(r, "ate_drift", "trans", "rmse")),
                     _fmt(_get(r, "rpe_distance", "trans_pct"), 2),
                     _fmt(_get(r, "rpe_distance", "rot_deg_per_100m"), 2)])
    return rows if len(rows) > 1 else None


def md_table(rows):
    out = ["| " + " | ".join(rows[0]) + " |", "|" + "---|" * len(rows[0])]
    out += ["| " + " | ".join(r) + " |" for r in rows[1:]]
    return "\n".join(out)


def text_table(rows):
    widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
    return "\n".join("  ".join(c.ljust(w) for c, w in zip(r, widths)) for r in rows)


def terminal_summary(summary):
    lines = []
    v = summary["verdict"]
    lines.append("结论: %s" % ("✓ 通过" if v["success"] else "✗ " + "；".join(v["reasons"])))
    t = accuracy_table(summary)
    if t:
        lines.append(text_table(t))
    g = summary.get("backend_gain")
    if g and "ate_improvement_pct" in g:
        def chg(p):
            return "降低 %.1f%%" % p if p >= 0 else "升高 %.1f%%" % -p
        lines.append("后端收益: ATE %.3f → %.3f m（%s），漂移 %.3f → %.3f m（%s）" % (
            g["ate_rmse_odometry"], g["ate_rmse_slam"], chg(g["ate_improvement_pct"]),
            g["drift_rmse_odometry"], g["drift_rmse_slam"], chg(g["drift_improvement_pct"])))
    c = summary.get("consistency")
    if c:
        ref = c.get("pose") or c.get("pos")
        lines.append("一致性: NEES/维度 = %.2f，%.0f%% 落在 95%% 区间（%s）" % (
            ref["mean_nees_per_dof"], ref["pct_in_95"], c.get("verdict", "")))
    lp = summary.get("loops")
    if lp and lp.get("candidates"):
        lines.append("回环: 候选 %d，接受 %d，精度 %s，召回 %s" % (
            lp["candidates"], lp["accepted"], _fmt(lp.get("precision"), 2), _fmt(lp.get("recall"), 2)))
    e = summary.get("efficiency", {})
    lat = _get(e, "latency_ms", "all")
    if lat:
        lines.append("效率: 每帧 %.1f ms（p99 %.1f，max %.1f），实时倍率 %s，峰值内存 %s MB，内存增长 %s MB/min" % (
            lat["mean"], lat["p99"], lat["max"], _fmt(e.get("realtime_factor"), 2),
            _fmt(_get(e, "memory", "peak_mb"), 0), _fmt(_get(e, "memory", "growth_mb_per_min"), 2)))
    return "\n".join(lines)


def markdown_report(summary, figs):
    m = summary.get("meta", {})
    L = ["# 评估报告：%s" % os.path.basename(os.path.normpath(summary["run"])), ""]
    L.append("- 版本 `%s`（%s），数据 `%s`" % (m.get("git_rev", "?"), m.get("build_type", "?"),
                                           os.path.basename(str(m.get("data", "?")))))
    L.append("- 主机 %s，%s，%s 线程" % (m.get("host", "?"), m.get("cpu", "?"), m.get("nproc", "?")))
    v = summary["verdict"]
    L.append("- **结论：%s**" % ("通过" if v["success"] else "未通过 —— " + "；".join(v["reasons"])))
    L.append("")

    if summary.get("gt"):
        g = summary["gt"]
        L += ["## 精度", "",
              "真值 %d 个位姿，%.1f s，%.1f m，%s。ATE 全程 %s 对齐；漂移误差只用开头 %.0f m 做 4 自由度对齐，"
              "反映在线使用时的累积误差。" % (
                  g["poses"], g["duration_s"], g["path_m"], "含姿态" if g["has_rot"] else "仅位置",
                  summary["options"]["align"], summary["options"]["head_m"]), ""]
        t = accuracy_table(summary)
        if t:
            L += [md_table(t), ""]
        for name, r in summary["trajectories"].items():
            segs = _get(r, "rpe_distance", "segments")
            if not segs:
                continue
            rows = [["段长 (m)", "样本", "平移 %", "平移 % 中位", "平移 % 最大", "旋转 °/100m"]]
            for s in segs:
                rows.append([_fmt(s["length_m"], 0), str(s["count"]), _fmt(s["trans_pct"], 2),
                             _fmt(s["trans_pct_median"], 2), _fmt(s["trans_pct_max"], 2),
                             _fmt(s.get("rot_deg_per_100m"), 2)])
            L += ["**%s 分段漂移**" % name, "", md_table(rows), ""]
        g = summary.get("backend_gain")
        if g and "ate_improvement_pct" in g:
            L += ["**后端 / 回环收益**：ATE %.3f → %.3f m（改善 %.1f%%），漂移误差 %.3f → %.3f m（改善 %.1f%%）。"
                  % (g["ate_rmse_odometry"], g["ate_rmse_slam"], g["ate_improvement_pct"],
                     g["drift_rmse_odometry"], g["drift_rmse_slam"], g["drift_improvement_pct"]), ""]
    elif summary.get("backend_gain"):
        g = summary["backend_gain"]
        L += ["## 后端修正量（无真值）", "",
              "odometry 与 slam 轨迹偏差：RMSE %.3f m，最大 %.3f m。" % (
                  g.get("odometry_vs_slam_rmse_m", 0), g.get("odometry_vs_slam_max_m", 0)), ""]

    rob = summary.get("robustness", {})
    L += ["## 稳定性", ""]
    rows = [["指标", "值"]]
    if "coverage" in rob:
        rows.append(["输出覆盖率", _fmt(100 * rob["coverage"], 1, "%")])
    if "init_time_s" in rob:
        rows.append(["初始化耗时", _fmt(rob["init_time_s"], 2, " s")])
    rows.append(["输出中断（> %.2fs）" % rob["gaps"]["threshold_s"],
                 "%d 次，共 %.2f s" % (rob["gaps"]["count"], rob["gaps"]["total_s"])])
    for name in ("odometry", "slam"):
        if "jumps_" + name in rob:
            rows.append(["%s 跳变" % name, str(rob["jumps_" + name]["count"])])
        if "divergence_" + name in rob:
            dv = rob["divergence_" + name]
            rows.append(["%s 发散" % name, "无" if not dv else "%.1f s / %.0f m 处，误差 %.2f m" % (
                dv["t_rel"], dv["dist_m"], dv["err_m"])])
        if "endpoint_" + name in rob:
            ep = rob["endpoint_" + name]
            rows.append(["%s 首尾漂移" % name, "%.3f m（%s）" % (ep["drift_m"], _fmt(ep["drift_pct"], 2, "%"))])
    L += [md_table(rows), ""]

    c = summary.get("consistency")
    if c:
        L += ["## 滤波一致性（NEES）", "",
              "理想值：NEES/维度 ≈ 1，约 95%% 的样本落在 95%% 卡方区间内。结论：**%s**。" % c.get("verdict", ""), ""]
        rows = [["分量", "维度", "NEES/维度 均值", "中位", "落在 95% 区间", "高于上界"]]
        for k in ("pos", "rot", "pose"):
            if k in c:
                r = c[k]
                rows.append([k, str(r["dof"]), _fmt(r["mean_nees_per_dof"], 2), _fmt(r["median_nees_per_dof"], 2),
                             _fmt(r["pct_in_95"], 1, "%"), _fmt(r["pct_above_95"], 1, "%")])
        L += [md_table(rows), ""]

    lp = summary.get("loops")
    if lp:
        L += ["## 回环", ""]
        rows = [["指标", "值"], ["候选（通过 Scan Context）", str(lp["candidates"])],
                ["接受", str(lp["accepted"])], ["ICP 拒绝", str(lp["rejected_icp"])],
                ["视觉拒绝", str(lp["rejected_visual"])]]
        for k, n in (("true_positive", "真回环"), ("false_positive", "**误回环**"),
                     ("precision", "精度"), ("rejected_but_true", "被拒绝的真回环"),
                     ("revisit_segments", "真值重访段"), ("recall", "召回")):
            if k in lp:
                rows.append([n, _fmt(lp[k], 2)])
        L += [md_table(rows), ""]

    e = summary.get("efficiency", {})
    if e:
        L += ["## 效率", ""]
        rows = [["指标", "值"]]
        rows.append(["实时倍率", _fmt(e.get("realtime_factor"), 2, "x")])
        for k, n in (("all", "每帧耗时"), ("lio", "LIO 帧"), ("vio", "VIO 帧")):
            x = _get(e, "latency_ms", k)
            if x:
                rows.append([n, "均值 %.1f / p95 %.1f / p99 %.1f / 最大 %.1f ms（%d 帧）" % (
                    x["mean"], x["p95"], x["p99"], x["max"], x["count"])])
        if "cycle_ms" in e:
            rows.append(["每 LiDAR 周期总耗时", "均值 %.1f / p99 %.1f ms，超预算（%.0f ms）%.1f%%" % (
                e["cycle_ms"]["mean"], e["cycle_ms"]["p99"], e["lidar_period_ms"], e["over_budget_pct"])])
        if "memory" in e:
            rows.append(["内存", "峰值 %.0f MB，后半程增长 %.2f MB/min" % (
                e["memory"]["peak_mb"], e["memory"]["growth_mb_per_min"])])
        if "cpu_pct" in e:
            rows.append(["CPU", "均值 %.0f%%，p95 %.0f%%" % (e["cpu_pct"]["mean"], e["cpu_pct"]["p95"])])
        if "dropped" in e:
            rows.append(["缓冲溢出丢弃", str(e["dropped"])])
        L += [md_table(rows), ""]

    h = summary.get("health", {})
    if h:
        L += ["## 内部健康度", ""]
        rows = [["指标", "值"]]
        for k, n in (("lio", "LIO"), ("vio", "VIO")):
            if k in h:
                r = h[k]
                rows.append(["%s 帧数 / 更新失败" % n, "%d / %.2f%%" % (r["frames"], r["update_fail_pct"])])
                if "num_valid" in r:
                    rows.append(["%s 有效残差数" % n, "中位 %.0f，p5 %.0f，最少 %.0f" % (
                        r["num_valid"]["median"], r["num_valid"]["p5"], r["num_valid"]["min"])])
                    rows.append(["%s 迭代 / 收敛率" % n, "%.2f / %.1f%%" % (r["iterations_mean"], r["converged_pct"])])
        rows.append(["退化帧占比", _fmt(h.get("degenerate_pct"), 2, "%")])
        if h.get("degenerate_dims_pct"):
            rows.append(["分维度退化", "，".join("%s %.1f%%" % (k, v) for k, v in h["degenerate_dims_pct"].items() if v > 0) or "无"])
        if "vio_track" in h:
            vt = h["vio_track"]
            rows.append(["视觉前端", "直接法 %.1f%% / 特征法 %.1f%%，切换 %d 次" % (
                vt["direct_pct"], vt["feature_pct"], vt["switches"])])
        conv = h.get("convergence", {})
        if "td_ms" in conv:
            rows.append(["td 最终值 / 尾段波动", "%.2f ms / %.3f ms" % (conv["td_ms"]["final"][0], conv["td_ms"]["tail_std"][0])])
        L += [md_table(rows), ""]

    if figs:
        L += ["## 图", ""]
        for title, fn in figs:
            L += ["**%s**" % title, "", "![](%s)" % fn, ""]
    return "\n".join(L)


# ---------------------------------------------------------------------------
# 图
# ---------------------------------------------------------------------------
def make_figures(summary, series, run, out_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    os.makedirs(out_dir, exist_ok=True)
    figs = []
    trajs = series["trajectories"]

    def save(fig, fn, title):
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, fn), dpi=110)
        plt.close(fig)
        figs.append((title, os.path.join(os.path.basename(out_dir), fn)))

    # 轨迹俯视图
    fig, ax = plt.subplots(figsize=(7, 6))
    has_gt = "gt_m" in trajs["odometry"]
    if has_gt:
        g = trajs["odometry"]["gt_m"]
        ax.plot(g.p[:, 0], g.p[:, 1], color=COLORS["gt"], lw=2, label="真值 / gt")
    for name, s in trajs.items():
        p = s["ate"]["aligned"].p if has_gt and "ate" in s else s["est"].p
        style = dict(ls="none", marker=".", ms=3) if name == "keyframes" else dict(lw=1)
        ax.plot(p[:, 0], p[:, 1], color=COLORS.get(name), label=name, **style)
    acc = [d for d in series.get("loops", []) if d.get("stage") == "accepted"]
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="best", fontsize=8)
    ax.grid(alpha=0.3)
    save(fig, "trajectory_xy.png", "轨迹俯视图（已对齐）")

    if has_gt:
        # 误差随时间
        fig, axes = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
        t0 = trajs["odometry"]["est"].t[0]
        for name in ("odometry", "slam"):
            if name not in trajs or "drift" not in trajs[name]:
                continue
            s = trajs[name]
            axes[0].plot(s["drift"]["t"] - t0, s["drift"]["trans"], color=COLORS[name], lw=1, label=name)
            axes[1].plot(s["ate"]["t"] - t0, s["ate"]["vertical"], color=COLORS[name], lw=1, label=name)
        for d in acc:
            for ax in axes:
                ax.axvline(d["t_from"] - t0, color="#999999", lw=0.6, ls="--")
        axes[0].set_ylabel("漂移误差 (m)\n开头对齐")
        axes[1].set_ylabel("竖直误差 (m)\n全程对齐")
        axes[1].set_xlabel("时间 (s)，虚线 = 接受的回环")
        for ax in axes:
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8)
        save(fig, "error_time.png", "误差随时间")

        # 分段漂移
        fig, ax = plt.subplots(figsize=(7, 4))
        w = 0.38
        for k, name in enumerate(("odometry", "slam")):
            segs = _get(summary, "trajectories", name, "rpe_distance", "segments")
            if not segs:
                continue
            x = np.arange(len(segs))
            ax.bar(x + (k - 0.5) * w, [s["trans_pct"] for s in segs], w, color=COLORS[name], label=name)
            ax.set_xticks(x)
            ax.set_xticklabels(["%g" % s["length_m"] for s in segs])
        ax.set_xlabel("段长 (m)")
        ax.set_ylabel("平移漂移 (%)")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3, axis="y")
        save(fig, "rpe_distance.png", "分段漂移率")

    # NEES
    if "nees" in series:
        ns = series["nees"]
        key = "pose" if "pose" in ns else "pos"
        c = summary["consistency"][key]
        fig, ax = plt.subplots(figsize=(9, 3.5))
        ax.semilogy(ns["t"] - ns["t"][0], ns[key] / c["dof"], lw=0.8, color="#2e86ab")
        ax.axhline(1.0, color="k", lw=0.8)
        ax.axhline(c["bound_hi"] / c["dof"], color="#d1495b", ls="--", lw=0.8)
        ax.axhline(c["bound_lo"] / c["dof"], color="#d1495b", ls="--", lw=0.8)
        ax.set_xlabel("时间 (s)")
        ax.set_ylabel("NEES / 维度（%s）" % key)
        ax.grid(alpha=0.3)
        save(fig, "nees.png", "滤波一致性（虚线为 95% 区间）")

    # 耗时与资源
    f = run.frames
    if f is not None and len(f.get("t", [])):
        fig, axes = plt.subplots(2, 1, figsize=(9, 5.5))
        t0 = f["t"][0]
        for m, name, col in ((1, "LIO", "#d1495b"), (2, "VIO", "#2e86ab")):
            sel = f["mode"] == m
            if sel.any():
                axes[0].plot(f["t"][sel] - t0, f["proc_ms"][sel], ".", ms=2, color=col, label=name)
        period = summary.get("efficiency", {}).get("lidar_period_ms")
        if period:
            axes[0].axhline(period, color="k", ls="--", lw=0.8, label="LiDAR 周期")
        axes[0].set_ylabel("耗时 (ms)")
        axes[0].legend(fontsize=8)
        axes[0].grid(alpha=0.3)
        ok = f["ok"] > 0
        lio = (f["mode"] == 1) & ok
        axes[1].plot(f["t"][lio] - t0, f["num_valid"][lio], lw=0.8, color="#d1495b", label="LIO 有效残差")
        deg = ok & (f["degenerate_mask"] > 0)
        if deg.any():
            ymax = np.max(f["num_valid"][lio]) if lio.any() else 1
            axes[1].plot(f["t"][deg] - t0, np.full(deg.sum(), ymax), "|", color="#edae49", label="退化帧")
        axes[1].set_ylabel("残差数")
        axes[1].set_xlabel("时间 (s)")
        axes[1].legend(fontsize=8)
        axes[1].grid(alpha=0.3)
        save(fig, "runtime.png", "逐帧耗时与观测健康度")

    r = run.resources
    if r is not None and len(r.get("wall_s", [])) > 2:
        fig, ax = plt.subplots(figsize=(9, 3.5))
        ax.plot(r["data_s"], r["rss_mb"], color="#2e86ab")
        ax.set_xlabel("数据时间 (s)")
        ax.set_ylabel("RSS (MB)")
        ax2 = ax.twinx()
        ax2.plot(r["data_s"], r["cpu_pct"], color="#999999", lw=0.6)
        ax2.set_ylabel("CPU (%)")
        ax.grid(alpha=0.3)
        save(fig, "resources.png", "内存（蓝）与 CPU（灰）")
    return figs


def setup_fonts():
    """中文标签：优先用系统里的 CJK 字体，找不到就退化为英文环境下的方框（不影响数据）。"""
    try:
        import matplotlib
        from matplotlib import font_manager
    except ImportError:
        return
    for name in ("Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Micro Hei", "WenQuanYi Zen Hei",
                 "SimHei", "Source Han Sans SC", "AR PL UMing CN"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            matplotlib.rcParams["font.sans-serif"] = [name] + matplotlib.rcParams["font.sans-serif"]
            break
    matplotlib.rcParams["axes.unicode_minus"] = False
