"""回环质量：对照真值判定每个回环的真 / 假，估计召回。

误回环是因子图的头号杀手（一条错误的边能拧坏整条轨迹），所以精度（precision）
比召回更重要；而召回决定了长距离漂移能被修正多少。两者都要看。

判定口径（均可通过参数调整）：
  真回环   两个关键帧时刻的真值位置相距 ≤ radius_m，且（有姿态真值时）
           测得的相对位姿与真值相对位姿相差 < (trans_m, rot_deg)
  重访段   时间相隔 ≥ min_dt 且真值位置相距 ≤ radius_m 的关键帧，按时间连续成段；
           召回 = 至少被一个真回环覆盖的重访段 / 重访段总数
"""
import numpy as np

from .accuracy import interpolate
from .geometry import quat_to_rot, rot_angle


def _gt_at(gt, t, max_gap):
    mask, g = interpolate(gt, np.asarray(t, dtype=float), max_gap)
    return mask, g


def evaluate(loops, keyframes, gt, radius_m=5.0, trans_m=1.0, rot_deg=5.0,
             min_dt=30.0, max_gap=0.5):
    if loops is None or "stage" not in loops or len(loops["stage"]) == 0:
        events = []
    else:
        events = list(range(len(loops["stage"])))
    stage = loops["stage"] if events else np.array([])
    res = {"candidates": len(events),
           "accepted": int(np.sum(stage == "accepted")) if events else 0,
           "rejected_icp": int(np.sum(stage == "rejected_icp")) if events else 0,
           "rejected_visual": int(np.sum(stage == "rejected_visual")) if events else 0}
    details = []
    if gt is None:
        return res, details

    if events:
        tf, tt = loops["t_from"], loops["t_to"]
        mf, gf = _gt_at(gt, tf, max_gap)
        mt, gtt = _gt_at(gt, tt, max_gap)
        both = mf & mt
        # 把子集结果映射回全长数组
        pf = np.full((len(tf), 3), np.nan)
        pt = np.full((len(tf), 3), np.nan)
        pf[mf], pt[mt] = gf.p, gtt.p
        Rf = Rt = None
        if gt.has_rot:
            Rf = np.full((len(tf), 3, 3), np.nan)
            Rt = np.full((len(tf), 3, 3), np.nan)
            Rf[mf], Rt[mt] = gf.R, gtt.R
        meas_t = np.stack([loops["dx"], loops["dy"], loops["dz"]], axis=1)
        meas_R = quat_to_rot(np.stack([loops["qx"], loops["qy"], loops["qz"], loops["qw"]], axis=1))
        for k in range(len(tf)):
            rec = {"stage": str(stage[k]), "t_from": float(tf[k]), "t_to": float(tt[k]),
                   "icp_fitness": float(loops["icp_fitness"][k])}
            if not both[k]:
                rec["truth"] = "unknown"
                details.append(rec)
                continue
            dist = float(np.linalg.norm(pf[k] - pt[k]))
            rec["gt_dist_m"] = dist
            truth = dist <= radius_m
            if gt.has_rot:
                # 测量的是 T_to_from（历史帧系下的当前帧）
                Rg = Rt[k].T @ Rf[k]
                tg = Rt[k].T @ (pf[k] - pt[k])
                et = float(np.linalg.norm(meas_t[k] - tg))
                er = float(np.degrees(rot_angle(Rg.T @ meas_R[k])))
                rec["rel_err_m"], rec["rel_err_deg"] = et, er
                truth = truth and et < trans_m and er < rot_deg
            rec["truth"] = "true" if truth else "false"
            details.append(rec)

        acc = [d for d in details if d["stage"] == "accepted" and d["truth"] != "unknown"]
        if acc:
            tp = sum(d["truth"] == "true" for d in acc)
            res["true_positive"] = tp
            res["false_positive"] = len(acc) - tp
            res["precision"] = tp / len(acc)
        rej = [d for d in details if d["stage"] != "accepted" and d["truth"] == "true"]
        res["rejected_but_true"] = len(rej)

    # 召回：基于关键帧真值位置找重访段
    if keyframes is not None and len(keyframes) > 2:
        mk, gk = _gt_at(gt, keyframes.t, max_gap)
        if mk.sum() > 2:
            P, T = gk.p, gk.t
            revisit = np.zeros(len(T), dtype=bool)
            for i in range(len(T)):
                old = T <= T[i] - min_dt
                if np.any(old):
                    revisit[i] = np.min(np.linalg.norm(P[old] - P[i], axis=1)) <= radius_m
            # 连续的重访关键帧合并成段
            segs, start = [], None
            for i, r in enumerate(revisit):
                if r and start is None:
                    start = i
                if (not r or i == len(T) - 1) and start is not None:
                    end = i if r else i - 1
                    segs.append((T[start], T[end]))
                    start = None
            res["revisit_segments"] = len(segs)
            if segs:
                hits = [d["t_from"] for d in details
                        if d["stage"] == "accepted" and d.get("truth") == "true"]
                covered = sum(any(a - 1.0 <= h <= b + 1.0 for h in hits) for a, b in segs)
                res["recall"] = covered / len(segs)
    return res, details
