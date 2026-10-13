"""多序列 / 多次重复的汇总，以及两次评测之间的对比。

单序列的数字没有意义 —— 参数调好可能只是过拟合了那一个序列。
同一序列跑多次的离散度同样重要：多线程时序带来的不确定性如果比一次改动的收益还大，
那次「改进」就只是噪声。
"""
import json
import os

import numpy as np

from . import io as lio
from .evaluate import evaluate_run
from .report import _fmt, _get, md_table, text_table, write_json


# ---------------------------------------------------------------------------
# 发现运行目录
# ---------------------------------------------------------------------------
def discover(root):
    """root 下每个子目录是一个序列：要么本身是运行目录，要么包含多个重复运行的子目录。
    返回 {序列名: [运行目录...]}。"""
    if lio.is_run_dir(root):
        return {os.path.basename(os.path.normpath(root)): [root]}
    seqs = {}
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        if not os.path.isdir(d) or name.startswith((".", "_")) or name == "eval":
            continue
        if lio.is_run_dir(d):
            seqs[name] = [d]
        else:
            reps = [os.path.join(d, r) for r in sorted(os.listdir(d))
                    if lio.is_run_dir(os.path.join(d, r))]
            if reps:
                seqs[name] = reps
    return seqs


def load_manifest(path):
    """{序列名: {gt: 路径, t_offset: 秒, extrinsic: "x y z [qx qy qz qw]", gt_format: ...}}。
    路径相对 manifest 所在目录。支持 JSON；装了 PyYAML 时也支持 YAML。"""
    with open(path) as f:
        text = f.read()
    if path.endswith((".yaml", ".yml")):
        import yaml
        data = yaml.safe_load(text)
    else:
        data = json.loads(text)
    base = os.path.dirname(os.path.abspath(path))
    for v in data.values():
        if "gt" in v and not os.path.isabs(v["gt"]):
            v["gt"] = os.path.join(base, v["gt"])
    return data


def find_gt(seq, gt_dir):
    if not gt_dir:
        return None
    for cand in (seq + ".tum", seq + ".txt", seq + ".csv",
                 os.path.join(seq, "groundtruth.tum"), os.path.join(seq, "groundtruth.txt"),
                 os.path.join(seq, "groundtruth.csv")):
        p = os.path.join(gt_dir, cand)
        if os.path.isfile(p):
            return p
    return None


# ---------------------------------------------------------------------------
# 指标抽取（batch 表格和 compare 共用同一套键）
# ---------------------------------------------------------------------------
# (键, 显示名, 取值路径, 越小越好?)
KEY_METRICS = [
    ("ate_odom", "ATE odom (m)", ("trajectories", "odometry", "ate", "trans", "rmse"), True),
    ("ate_slam", "ATE slam (m)", ("trajectories", "slam", "ate", "trans", "rmse"), True),
    ("drift_slam", "漂移 slam (m)", ("trajectories", "slam", "ate_drift", "trans", "rmse"), True),
    ("rpe_pct", "RPE %", ("trajectories", "odometry", "rpe_distance", "trans_pct"), True),
    ("rot_deg", "旋转 (°)", ("trajectories", "slam", "ate", "rot_deg", "rmse"), True),
    ("nees", "NEES/dof", None, None),
    ("loop_prec", "回环精度", ("loops", "precision"), False),
    ("loop_recall", "回环召回", ("loops", "recall"), False),
    ("latency_p99", "p99 (ms)", ("efficiency", "latency_ms", "all", "p99"), True),
    ("rt_factor", "实时倍率", ("efficiency", "realtime_factor"), False),
    ("mem_growth", "内存增长 MB/min", ("efficiency", "memory", "growth_mb_per_min"), True),
]


def extract(summary):
    out = {}
    for key, _, path, _ in KEY_METRICS:
        if key == "nees":
            c = summary.get("consistency") or {}
            ref = c.get("pose") or c.get("pos")
            out[key] = ref["mean_nees_per_dof"] if ref else None
            continue
        out[key] = _get(summary, *path)
    if out["ate_slam"] is None:   # 关了后端时 slam 轨迹不存在
        out["ate_slam"] = out["ate_odom"]
        out["drift_slam"] = _get(summary, "trajectories", "odometry", "ate_drift", "trans", "rmse")
        out["rot_deg"] = _get(summary, "trajectories", "odometry", "ate", "rot_deg", "rmse")
    out["success"] = summary["verdict"]["success"]
    return out


def _agg(values):
    v = np.array([x for x in values if x is not None], dtype=float)
    if len(v) == 0:
        return None
    return {"mean": float(np.mean(v)), "std": float(np.std(v)), "min": float(np.min(v)),
            "max": float(np.max(v)), "n": int(len(v))}


# ---------------------------------------------------------------------------
# batch
# ---------------------------------------------------------------------------
def run_batch(root, opt_factory, gt_dir=None, manifest=None, out_dir=None, plots=True, log=print):
    from .report import make_figures, markdown_report, setup_fonts
    seqs = discover(root)
    if not seqs:
        raise lio.EvalError("%s 下没有找到运行目录（含 odometry.tum 的目录）" % root)
    out_dir = out_dir or os.path.join(root, "eval")
    os.makedirs(out_dir, exist_ok=True)
    if plots:
        setup_fonts()

    per_seq = {}
    for seq, runs in seqs.items():
        spec = (manifest or {}).get(seq, {})
        gt_path = spec.get("gt") or find_gt(seq, gt_dir)
        gt = lio.load_trajectory(gt_path, spec.get("gt_format", "auto")) if gt_path else None
        opt = opt_factory(spec)
        rows = []
        for rd in runs:
            try:
                run = lio.Run(rd)
                summary, series = evaluate_run(run, gt, opt)
            except lio.EvalError as e:
                log("  [%s] %s: %s" % (seq, os.path.basename(rd), e))
                rows.append({"run": rd, "error": str(e), "success": False})
                continue
            ev = os.path.join(rd, "eval")
            os.makedirs(ev, exist_ok=True)
            figs = make_figures(summary, series, run, os.path.join(ev, "figs")) if plots else []
            write_json(os.path.join(ev, "summary.json"), summary)
            with open(os.path.join(ev, "report.md"), "w") as f:
                f.write(markdown_report(summary, figs))
            m = extract(summary)
            m["run"] = rd
            m["reasons"] = summary["verdict"]["reasons"]
            rows.append(m)
            log("  [%s] %s: ATE odom %s / slam %s m%s" % (
                seq, os.path.basename(rd), _fmt(m["ate_odom"]), _fmt(m["ate_slam"]),
                "" if m["success"] else "  ✗ " + "；".join(m["reasons"])))
        ok_rows = [r for r in rows if "error" not in r]
        agg = {k: _agg([r.get(k) for r in ok_rows]) for k, _, _, _ in KEY_METRICS}
        agg["success_rate"] = float(np.mean([r["success"] for r in rows])) if rows else 0.0
        agg["repeats"] = len(rows)
        agg["has_gt"] = gt is not None
        if len(ok_rows) > 1:
            agg["spread"] = repeat_spread([r["run"] for r in ok_rows])
        per_seq[seq] = {"runs": rows, "aggregate": agg}

    overall = overall_stats(per_seq)
    result = {"root": root, "sequences": per_seq, "overall": overall}
    write_json(os.path.join(out_dir, "batch_summary.json"), result)
    figs = batch_figures(per_seq, out_dir) if plots else []
    with open(os.path.join(out_dir, "batch_report.md"), "w") as f:
        f.write(batch_markdown(result, figs))
    return result


def repeat_spread(run_dirs):
    """同一序列多次运行的轨迹离散度：每次运行与第一次的 slam（或 odometry）轨迹的 RMS 距离。
    不需要真值，直接度量系统的不确定性。"""
    from .accuracy import associate, compute_alignment
    runs = [lio.Run(d) for d in run_dirs]
    ref = runs[0].slam if runs[0].slam is not None else runs[0].odometry
    vals = []
    for r in runs[1:]:
        est = r.slam if r.slam is not None else r.odometry
        try:
            _, e, g = associate(est, ref, 0.05)
        except lio.EvalError:
            continue
        R, t, _ = compute_alignment(e, g, "posyaw")
        vals.append(float(np.sqrt(np.mean(np.sum((e.transformed(R, t).p - g.p) ** 2, axis=1)))))
    return {"rms_to_first_m": vals, "max_m": max(vals) if vals else None}


def overall_stats(per_seq):
    with_gt = {s: v for s, v in per_seq.items() if v["aggregate"]["has_gt"]}
    out = {"sequences": len(per_seq), "with_gt": len(with_gt)}
    all_runs = [r for v in per_seq.values() for r in v["runs"]]
    out["success_rate"] = float(np.mean([r["success"] for r in all_runs])) if all_runs else 0.0
    for key in ("ate_odom", "ate_slam", "drift_slam", "rpe_pct", "latency_p99", "rt_factor"):
        vals = [v["aggregate"][key]["mean"] for v in per_seq.values()
                if v["aggregate"].get(key)]
        if vals:
            worst = max(vals) if key != "rt_factor" else min(vals)
            out[key] = {"mean": float(np.mean(vals)), "median": float(np.median(vals)),
                        "worst": float(worst)}
    return out


def batch_table(per_seq):
    rows = [["序列", "次数", "成功率", "ATE odom", "ATE slam", "漂移 slam", "RPE %",
             "NEES/dof", "回环精度", "p99 ms", "实时倍率"]]

    def cell(agg, key, nd=3):
        a = agg.get(key)
        if not a:
            return "—"
        if a["n"] > 1:
            return ("%." + str(nd) + "f±%." + str(nd) + "f") % (a["mean"], a["std"])
        return _fmt(a["mean"], nd)

    for seq, v in per_seq.items():
        a = v["aggregate"]
        rows.append([seq, str(a["repeats"]), _fmt(100 * a["success_rate"], 0, "%"),
                     cell(a, "ate_odom"), cell(a, "ate_slam"), cell(a, "drift_slam"),
                     cell(a, "rpe_pct", 2), cell(a, "nees", 2), cell(a, "loop_prec", 2),
                     cell(a, "latency_p99", 1), cell(a, "rt_factor", 2)])
    return rows


def batch_markdown(result, figs):
    o = result["overall"]
    L = ["# 批量评估：%s" % result["root"], "",
         "%d 个序列（%d 个有真值），总成功率 **%.0f%%**。" % (
             o["sequences"], o["with_gt"], 100 * o["success_rate"]), ""]
    rows = [["指标", "均值", "中位", "最差"]]
    for key, name in (("ate_odom", "ATE odom (m)"), ("ate_slam", "ATE slam (m)"),
                      ("drift_slam", "漂移 slam (m)"), ("rpe_pct", "RPE (%)"),
                      ("latency_p99", "p99 耗时 (ms)"), ("rt_factor", "实时倍率")):
        if key in o:
            rows.append([name, _fmt(o[key]["mean"]), _fmt(o[key]["median"]), _fmt(o[key]["worst"])])
    L += [md_table(rows), "", "## 各序列", "", "多次运行时显示 均值±标准差。", "",
          md_table(batch_table(result["sequences"])), ""]
    fails = [(s, r) for s, v in result["sequences"].items() for r in v["runs"] if not r["success"]]
    if fails:
        L += ["## 未通过", ""]
        for s, r in fails:
            L.append("- %s / %s：%s" % (s, os.path.basename(r["run"]),
                                       r.get("error") or "；".join(r.get("reasons", []))))
        L.append("")
    spreads = [(s, v["aggregate"]["spread"]) for s, v in result["sequences"].items()
               if v["aggregate"].get("spread")]
    if spreads:
        L += ["## 重复性", "", "同一序列多次运行之间的轨迹差异（与第一次运行比较，不需要真值）。", ""]
        for s, sp in spreads:
            L.append("- %s：最大 %s m" % (s, _fmt(sp["max_m"])))
        L.append("")
    for title, fn in figs:
        L += ["**%s**" % title, "", "![](%s)" % fn, ""]
    return "\n".join(L)


def batch_figures(per_seq, out_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    figs = []
    fig, ax = plt.subplots(figsize=(6.5, 4))
    drawn = False
    for key, name, col in (("ate_odom", "odometry", "#d1495b"), ("ate_slam", "slam", "#2e86ab")):
        vals = sorted(r[key] for v in per_seq.values() for r in v["runs"] if r.get(key) is not None)
        if not vals:
            continue
        y = np.arange(1, len(vals) + 1) / len(vals)
        ax.step(vals, y, where="post", color=col, label=name)
        drawn = True
    if drawn:
        ax.set_xscale("log")
        ax.set_xlabel("ATE RMSE (m)")
        ax.set_ylabel("运行占比（≤ x）")
        ax.grid(alpha=0.3, which="both")
        ax.legend(fontsize=8)
        fig.tight_layout()
        os.makedirs(os.path.join(out_dir, "figs"), exist_ok=True)
        fig.savefig(os.path.join(out_dir, "figs", "ate_cdf.png"), dpi=110)
        figs.append(("ATE 累积分布（曲线越靠左上越好）", "figs/ate_cdf.png"))
    plt.close(fig)
    return figs


def print_batch(result):
    print(text_table(batch_table(result["sequences"])))
    o = result["overall"]
    msg = "成功率 %.0f%%" % (100 * o["success_rate"])
    if "ate_slam" in o:
        msg += "，平均 ATE slam %.3f m（最差 %.3f）" % (o["ate_slam"]["mean"], o["ate_slam"]["worst"])
    print(msg)


# ---------------------------------------------------------------------------
# compare
# ---------------------------------------------------------------------------
def _load_metrics(path):
    """batch 目录 / batch_summary.json / 单次运行目录 → {序列: {键: (均值, 标准差)}}。"""
    cands = [path, os.path.join(path, "batch_summary.json"),
             os.path.join(path, "eval", "batch_summary.json"),
             os.path.join(path, "eval", "summary.json"), os.path.join(path, "summary.json")]
    for c in cands:
        if os.path.isfile(c):
            with open(c) as f:
                data = json.load(f)
            if "sequences" in data:
                return {s: {k: ((v["aggregate"].get(k) or {}).get("mean"),
                                (v["aggregate"].get(k) or {}).get("std"))
                            for k, _, _, _ in KEY_METRICS}
                        for s, v in data["sequences"].items()}
            m = extract(data)
            name = os.path.basename(os.path.normpath(path))
            return {name: {k: (m[k], None) for k, _, _, _ in KEY_METRICS}}
    raise lio.EvalError("%s 下没有 batch_summary.json 或 eval/summary.json，先运行 lvi_eval batch/run" % path)


def compare(base, new, threshold_pct=5.0):
    """返回 (表格行, 回归列表)。变化超过阈值、且超过两边重复运行的标准差之和，才算显著。"""
    A, B = _load_metrics(base), _load_metrics(new)
    common = [s for s in A if s in B]
    if not common and len(A) == 1 and len(B) == 1:
        common = [(list(A)[0], list(B)[0])]
    else:
        common = [(s, s) for s in common]
    if not common:
        raise lio.EvalError("两次评测没有共同的序列")
    keys = [(k, n, smaller) for k, n, _, smaller in KEY_METRICS if smaller is not None or k == "nees"]
    rows = [["序列", "指标", "基线", "新版", "变化", ""]]
    regressions = []
    for sa, sb in common:
        for k, name, smaller in keys:
            (a, sda), (b, sdb) = A[sa][k], B[sb][k]
            if a is None or b is None:
                continue
            delta = 100.0 * (b - a) / abs(a) if abs(a) > 1e-12 else 0.0
            noise = (sda or 0) + (sdb or 0)
            significant = abs(delta) >= threshold_pct and abs(b - a) > noise
            if k == "nees":   # 越接近 1 越好
                better = abs(b - 1) < abs(a - 1)
            else:
                better = (b < a) if smaller else (b > a)
            mark = ""
            if significant:
                mark = "✓ 改善" if better else "✗ 退步"
                if not better:
                    regressions.append((sa, name, a, b, delta))
            rows.append([sa if sa == sb else "%s→%s" % (sa, sb), name, _fmt(a), _fmt(b), "%+.1f%%" % delta, mark])
    return rows, regressions
