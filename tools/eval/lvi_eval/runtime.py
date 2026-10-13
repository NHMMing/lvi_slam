"""效率与内部健康度：只依赖 frames.csv / resources.csv / run_meta.json，不需要真值。"""
import numpy as np

DIM_NAMES = ["roll", "pitch", "yaw", "x", "y", "z"]


def _pct(x, q):
    return float(np.percentile(x, q)) if len(x) else None


def efficiency(run):
    out = {}
    meta = run.meta or {}
    for k in ("realtime_factor", "wall_s", "data_span_s", "peak_rss_mb", "realtime"):
        if k in meta:
            out[k] = meta[k]
    if "dropped" in meta:
        out["dropped"] = meta["dropped"]
    f = run.frames
    if f is not None and len(f.get("t", [])):
        ms, mode = f["proc_ms"], f["mode"]
        lio_t = f["t"][mode == 1]
        period = float(np.median(np.diff(lio_t))) if len(lio_t) > 2 else 0.1
        out["lidar_period_ms"] = 1000 * period
        out["latency_ms"] = {}
        for name, sel in (("all", np.ones(len(ms), bool)), ("lio", mode == 1), ("vio", mode == 2)):
            x = ms[sel]
            if len(x) == 0:
                continue
            out["latency_ms"][name] = {
                "mean": float(np.mean(x)), "median": float(np.median(x)),
                "p95": _pct(x, 95), "p99": _pct(x, 99), "max": float(np.max(x)), "count": int(len(x))}
        # 一个 LiDAR 周期内要做完 LIO + VIO：按「每个 LiDAR 周期总耗时」算超预算比例
        if len(lio_t) > 2:
            bins = np.searchsorted(lio_t, f["t"], side="right")
            per_cycle = np.bincount(bins, weights=ms)
            per_cycle = per_cycle[1:len(lio_t)]
            out["cycle_ms"] = {"mean": float(np.mean(per_cycle)), "p99": _pct(per_cycle, 99),
                               "max": float(np.max(per_cycle))}
            out["over_budget_pct"] = float(100.0 * np.mean(per_cycle > 1000 * period))
    r = run.resources
    if r is not None and len(r.get("wall_s", [])) > 4:
        rss, cpu = r["rss_mb"], r["cpu_pct"]
        x = r["data_s"] if np.ptp(r["data_s"]) > 0 else r["wall_s"]
        half = len(x) // 2
        # 只用后半段拟合增长率，避开建图初期的正常上升
        slope = np.polyfit(x[half:], rss[half:], 1)[0] * 60.0 if np.ptp(x[half:]) > 0 else 0.0
        out["memory"] = {"peak_mb": float(np.max(rss)), "final_mb": float(rss[-1]),
                         "growth_mb_per_min": float(slope)}
        out["cpu_pct"] = {"mean": float(np.mean(cpu)), "p95": _pct(cpu, 95)}
    return out


def health(run):
    f = run.frames
    if f is None or len(f.get("t", [])) == 0:
        return {}
    out = {}
    mode, ok = f["mode"], f["ok"] > 0
    for name, m in (("lio", 1), ("vio", 2)):
        sel = mode == m
        n = int(sel.sum())
        if n == 0:
            continue
        nv = f["num_valid"][sel & ok]
        rec = {"frames": n, "update_fail_pct": float(100.0 * np.mean(~ok[sel]))}
        if len(nv):
            rec["num_valid"] = {"median": float(np.median(nv)), "p5": _pct(nv, 5),
                                "min": float(np.min(nv))}
            rec["iterations_mean"] = float(np.mean(f["iterations"][sel & ok]))
            rec["converged_pct"] = float(100.0 * np.mean(f["converged"][sel & ok] > 0))
        out[name] = rec

    mask = f["degenerate_mask"].astype(int)
    okm = mask[ok]
    out["degenerate_pct"] = float(100.0 * np.mean(okm > 0)) if len(okm) else 0.0
    out["degenerate_dims_pct"] = {
        DIM_NAMES[d]: float(100.0 * np.mean((okm >> d) & 1)) for d in range(6)} if len(okm) else {}

    vt = f["vio_track"][mode == 2]
    vt = vt[vt >= 0]
    if len(vt):
        out["vio_track"] = {"direct_pct": float(100.0 * np.mean(vt == 0)),
                            "feature_pct": float(100.0 * np.mean(vt == 1)),
                            "switches": int(np.sum(np.diff(vt) != 0))}

    # td / bias 收敛：看最后 30% 的波动，以及与最终值的偏差
    t = f["t"][ok]
    if len(t) > 10:
        tail = t >= t[0] + 0.7 * (t[-1] - t[0])
        conv = {}
        for key, cols in (("td_ms", ["td"]), ("bg", ["bg_x", "bg_y", "bg_z"]),
                          ("ba", ["ba_x", "ba_y", "ba_z"])):
            v = np.stack([f[c][ok] for c in cols], axis=1)
            scale = 1000.0 if key == "td_ms" else 1.0
            conv[key] = {"final": (v[-1] * scale).tolist(),
                         "tail_std": (np.std(v[tail], axis=0) * scale).tolist()}
        out["convergence"] = conv
    return out
