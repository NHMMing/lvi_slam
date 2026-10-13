"""lvi_eval 单元测试。只用标准库 unittest：python3 -m unittest discover -s tools/eval/tests"""
import io
import os
import shutil
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from lvi_eval import accuracy, consistency, geometry as G, loops  # noqa: E402
from lvi_eval.cli import main  # noqa: E402
from lvi_eval.geometry import Trajectory  # noqa: E402
from lvi_eval.io import Run  # noqa: E402
import synth  # noqa: E402


class TestGeometry(unittest.TestCase):
    def test_quat_roundtrip(self):
        rng = np.random.RandomState(0)
        q = rng.randn(50, 4)
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        q[q[:, 3] < 0] *= -1
        np.testing.assert_allclose(G.rot_to_quat(G.quat_to_rot(q)), q, atol=1e-12)

    def test_exp_log_roundtrip_including_near_pi(self):
        rng = np.random.RandomState(1)
        axis = rng.randn(20, 3)
        axis /= np.linalg.norm(axis, axis=1, keepdims=True)
        angles = np.concatenate([rng.uniform(0, 3.0, 17), [1e-9, np.pi - 1e-6, np.pi - 1e-3]])
        phi = axis * angles[:, None]
        np.testing.assert_allclose(G.so3_log(G.so3_exp(phi)), phi, atol=1e-6)

    def test_umeyama_recovers_sim3(self):
        rng = np.random.RandomState(2)
        src = rng.randn(100, 3) * 10
        R = G.so3_exp(np.array([0.3, -0.2, 1.1]))
        t, s = np.array([5.0, -3.0, 2.0]), 1.7
        dst = s * src @ R.T + t
        R2, t2, s2 = G.umeyama(src, dst, with_scale=True)
        np.testing.assert_allclose(R2, R, atol=1e-9)
        np.testing.assert_allclose(t2, t, atol=1e-9)
        self.assertAlmostEqual(s2, s, places=9)

    def test_posyaw_recovers_yaw_only(self):
        rng = np.random.RandomState(3)
        src = rng.randn(100, 3) * 10
        R, t = G.rot_z(0.7), np.array([1.0, 2.0, 3.0])
        R2, t2, _ = G.align_pos_yaw(src, src @ R.T + t)
        np.testing.assert_allclose(R2, R, atol=1e-9)
        np.testing.assert_allclose(t2, t, atol=1e-9)

    def test_interpolate_midpoint(self):
        tr = Trajectory([0.0, 1.0], [[0, 0, 0], [2, 0, 0]], G.so3_exp(np.array([[0, 0, 0], [0, 0, 1.0]])))
        mask, out = accuracy.interpolate(tr, np.array([0.0, 0.5, 1.0, 1.5]), max_gap=2.0)
        np.testing.assert_array_equal(mask, [True, True, True, False])
        np.testing.assert_allclose(out.p[1], [1, 0, 0])
        np.testing.assert_allclose(G.so3_log(out.R[1]), [0, 0, 0.5], atol=1e-12)


class TestAccuracy(unittest.TestCase):
    def setUp(self):
        self.gt = synth.figure_eight()

    def test_ate_zero_under_gauge_transform(self):
        est = self.gt.transformed(G.rot_z(1.2), np.array([10.0, -4.0, 2.0]))
        res, _ = accuracy.ate(est, self.gt, "posyaw")
        self.assertLess(res["trans"]["rmse"], 1e-9)
        self.assertLess(res["rot_deg"]["rmse"], 1e-6)

    def test_posyaw_does_not_hide_roll_error(self):
        # 整条轨迹带 2° roll 偏差：6 自由度对齐会把它对齐掉，4 自由度不会
        Rx = G.so3_exp(np.array([np.radians(2.0), 0, 0]))
        est = self.gt.transformed(Rx, np.zeros(3))
        a4, _ = accuracy.ate(est, self.gt, "posyaw")
        a6, _ = accuracy.ate(est, self.gt, "se3")
        self.assertLess(a6["rot_deg"]["rmse"], 1e-5)
        self.assertAlmostEqual(a4["rot_deg"]["rmse"], 2.0, places=3)

    def test_rpe_distance_measures_scale_drift(self):
        # 直线轨迹：弦长 = 路程，2% 尺度漂移恰好对应 2% 的分段漂移
        t = np.arange(0, 100, 0.1)
        line = Trajectory(t, np.stack([t, 0.5 * t, 0 * t], 1), np.tile(G.rot_z(0.46), (len(t), 1, 1)))
        est = synth.with_drift(line, trans_pct=2.0)
        res = accuracy.rpe_distance(est, line, [10, 20], est)
        self.assertAlmostEqual(res["trans_pct"], 2.0, delta=0.05)
        self.assertLess(res["rot_deg_per_100m"], 1e-6)

    def test_rpe_distance_measures_yaw_drift(self):
        est = synth.with_drift(self.gt, trans_pct=0.0, yaw_rate_deg_per_100m=1.0)
        res = accuracy.rpe_distance(est, self.gt, [20], est)
        self.assertAlmostEqual(res["rot_deg_per_100m"], 1.0, delta=0.02)

    def test_position_only_gt(self):
        gt_pos = Trajectory(self.gt.t, self.gt.p)
        est = synth.with_drift(self.gt, trans_pct=1.0)
        _, e, g = accuracy.associate(est, gt_pos)
        res, s = accuracy.ate(e, g)
        self.assertNotIn("rot_deg", res)
        rpe = accuracy.rpe_distance(e, g, [20], s["aligned"])
        self.assertGreater(rpe["trans_pct"], 0.3)

    def test_lever_arm(self):
        # 真值测的是 IMU 前方 0.5m 的点；给对外参后误差应为 0
        R_bx, t_bx = np.eye(3), np.array([0.5, 0.0, 0.2])
        gt_x = self.gt.with_body_offset(R_bx, t_bx)
        res_wrong, _ = accuracy.ate(self.gt, gt_x, "posyaw")
        res_right, _ = accuracy.ate(self.gt.with_body_offset(R_bx, t_bx), gt_x, "posyaw")
        self.assertGreater(res_wrong["trans"]["rmse"], 0.1)
        self.assertLess(res_right["trans"]["rmse"], 1e-9)

    def test_time_offset_error_message(self):
        est = self.gt.with_time_offset(-900.0)   # 开机时钟 vs UNIX 时间
        with self.assertRaises(Exception) as ctx:
            accuracy.associate(est, self.gt)
        self.assertIn("--t-offset", str(ctx.exception))


class TestConsistency(unittest.TestCase):
    def test_chi2_ppf(self):
        # 参考值来自 scipy.stats.chi2.ppf
        self.assertAlmostEqual(consistency.chi2_ppf(0.975, 3), 9.3484, places=3)
        self.assertAlmostEqual(consistency.chi2_ppf(0.025, 3), 0.2158, places=3)
        self.assertAlmostEqual(consistency.chi2_ppf(0.025, 6), 1.2373, places=3)
        self.assertAlmostEqual(consistency.chi2_ppf(0.975, 6), 14.4494, places=3)

    def test_nees_is_one_for_matched_covariance(self):
        gt = synth.figure_eight(duration=300.0)
        sp, sr = 0.05, 0.01
        est, _, _ = synth.with_noise(gt, sp, sr, seed=4)
        n = len(gt)
        P = np.tile(np.diag([sr ** 2] * 3 + [sp ** 2] * 3), (n, 1, 1))
        frames = {"t": est.t, "ok": np.ones(n)}
        for i in range(6):
            for j in range(i, 6):
                frames["cov_%d%d" % (i, j)] = P[:, i, j]
        res, _ = consistency.nees(est, gt, frames)
        self.assertAlmostEqual(res["pose"]["mean_nees_per_dof"], 1.0, delta=0.08)
        self.assertAlmostEqual(res["pose"]["pct_in_95"], 95.0, delta=2.0)
        # 协方差小 4 倍 → 过度自信
        frames2 = dict(frames)
        for k in list(frames2):
            if k.startswith("cov_"):
                frames2[k] = frames[k] / 4
        res2, _ = consistency.nees(est, gt, frames2)
        self.assertAlmostEqual(res2["pose"]["mean_nees_per_dof"], 4.0, delta=0.3)
        self.assertIn("自信", res2["verdict"])


class TestLoops(unittest.TestCase):
    def test_precision_and_recall(self):
        gt = synth.figure_eight(duration=120.0)
        kfs = gt.subset(np.arange(0, len(gt), 10))
        tf, tt = gt.t[-5], gt.t[3]                      # 回到起点：真回环
        i_f, i_t = len(gt) - 5, 3
        Rr = gt.R[i_t].T @ gt.R[i_f]
        tr = gt.R[i_t].T @ (gt.p[i_f] - gt.p[i_t])
        q = G.rot_to_quat(Rr)
        far = len(gt) // 4                              # 远离的两点：误回环
        data = {"stage": np.array(["accepted", "accepted", "rejected_icp"]),
                "t_from": np.array([tf, gt.t[far + 400], gt.t[-3]]),
                "t_to": np.array([tt, gt.t[far], gt.t[2]]),
                "icp_fitness": np.array([0.1, 0.2, 0.5]),
                "dx": np.array([tr[0], 0, 0]), "dy": np.array([tr[1], 0, 0]), "dz": np.array([tr[2], 0, 0]),
                "qx": np.array([q[0], 0, 0]), "qy": np.array([q[1], 0, 0]),
                "qz": np.array([q[2], 0, 0]), "qw": np.array([q[3], 1, 1])}
        res, det = loops.evaluate(data, kfs, gt, radius_m=5.0)
        self.assertEqual(res["true_positive"], 1)
        self.assertEqual(res["false_positive"], 1)
        self.assertAlmostEqual(res["precision"], 0.5)
        self.assertGreaterEqual(res["revisit_segments"], 1)
        self.assertGreater(res["recall"], 0)


class TestCli(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="lvi_eval_test_")
        self.gt = synth.figure_eight(duration=150.0)
        from lvi_eval.io import write_tum
        self.gt_path = os.path.join(self.tmp, "gt", "seqA.tum")
        os.makedirs(os.path.dirname(self.gt_path))
        write_tum(self.gt_path, self.gt)

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _make(self, name, drift, seed=0):
        odo = synth.with_drift(self.gt, trans_pct=drift, yaw_rate_deg_per_100m=0.5)
        slam, _, _ = synth.with_noise(self.gt, 0.02, 0.002, seed=seed)
        kf = slam.subset(np.arange(0, len(slam), 10))
        return synth.fake_run(os.path.join(self.tmp, name), odo, slam, kf)

    def _quiet(self, argv):
        buf = io.StringIO()
        with redirect_stdout(buf):
            code = main(argv)
        return code, buf.getvalue()

    def test_run_with_and_without_gt(self):
        rd = self._make("runs/seqA", 1.0)
        code, out = self._quiet(["run", "--run", rd, "--gt", self.gt_path])
        self.assertIn("后端收益", out)
        for fn in ("summary.json", "report.md", "figs/trajectory_xy.png", "figs/nees.png"):
            self.assertTrue(os.path.isfile(os.path.join(rd, "eval", fn)), fn)
        code, out = self._quiet(["run", "--run", rd, "--no-plots", "--closed-loop"])
        self.assertIn("效率", out)

    def test_batch_with_repeats_and_compare(self):
        for k in range(3):
            self._make("base/seqA/rep%d" % k, 1.0, seed=k)
            self._make("new/seqA/rep%d" % k, 3.0, seed=k)
        gt_dir = os.path.dirname(self.gt_path)
        for root in ("base", "new"):
            code, _ = self._quiet(["batch", os.path.join(self.tmp, root), "--gt-dir", gt_dir, "--no-plots"])
            self.assertTrue(os.path.isfile(os.path.join(self.tmp, root, "eval", "batch_summary.json")))
        code, out = self._quiet(["compare", os.path.join(self.tmp, "base"), os.path.join(self.tmp, "new"),
                                 "--fail-on-regression"])
        self.assertEqual(code, 3)         # 里程计漂移 1% → 3%，必须报退步
        self.assertIn("退步", out)

    def test_batch_counts_crashed_runs(self):
        # eval_batch.sh 在 lvi_offline 崩溃时只留一个占位 run_meta.json
        self._make("runs/seqA/rep0", 1.0)
        crashed = os.path.join(self.tmp, "runs", "seqA", "rep1")
        os.makedirs(crashed)
        with open(os.path.join(crashed, "run_meta.json"), "w") as f:
            f.write('{"crashed": true, "exit_code": 134}\n')
        code, out = self._quiet(["batch", os.path.join(self.tmp, "runs"),
                                 "--gt-dir", os.path.dirname(self.gt_path), "--no-plots"])
        self.assertIn("异常退出", out)
        self.assertIn("成功率 50%", out)


if __name__ == "__main__":
    unittest.main()
