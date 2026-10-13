"""命令行入口：lvi_eval run | batch | compare。"""
import argparse
import os
import sys

from . import __version__
from . import io as lio
from .evaluate import Options


def _add_eval_options(p):
    g = p.add_argument_group("真值与对齐")
    g.add_argument("--gt-format", default="auto", choices=["auto", "tum", "euroc"],
                   help="真值格式；TUM 只有 4 列 (t x y z) 时视为仅位置真值")
    g.add_argument("--t-offset", type=float, default=0.0,
                   help="加到估计时间戳上的偏移 (s)，用于时钟域不一致的数据")
    g.add_argument("--extrinsic", default=None,
                   help="真值点在 IMU 系下的位姿：'x y z'（杆臂）或 'x y z qx qy qz qw'")
    g.add_argument("--max-gap", type=float, default=0.5, help="真值插值允许的最大样本间隔 (s)")
    g.add_argument("--align", default="posyaw", choices=["posyaw", "se3", "sim3", "none"],
                   help="全程 ATE 的对齐方式（默认 4 自由度 yaw+xyz）")
    g.add_argument("--head", type=float, default=20.0, dest="head_m",
                   help="漂移误差只用开头这么多米对齐（默认 20）")
    g.add_argument("--rpe-lengths", default=None,
                   help="分段漂移的段长，逗号分隔，单位 m（默认按路程自动选）")
    g = p.add_argument_group("稳定性判据")
    g.add_argument("--jump-m", type=float, default=0.3)
    g.add_argument("--jump-deg", type=float, default=3.0)
    g.add_argument("--fail-abs", type=float, default=5.0, help="发散阈值：漂移误差 > max(abs, rel×路程)")
    g.add_argument("--fail-rel", type=float, default=0.05)
    g.add_argument("--min-coverage", type=float, default=0.95)
    g.add_argument("--success-ate", type=float, default=None, help="成功判据：ATE RMSE 上限 (m)")
    g.add_argument("--closed-loop", action="store_true", help="序列首尾重合：额外输出端点漂移")
    g = p.add_argument_group("回环判定")
    g.add_argument("--loop-radius", type=float, default=5.0)
    g.add_argument("--loop-trans", type=float, default=1.0)
    g.add_argument("--loop-rot", type=float, default=5.0)
    g.add_argument("--loop-min-dt", type=float, default=30.0)
    p.add_argument("--no-plots", action="store_true", help="不画图（快，适合 CI）")


def _options(args, spec=None):
    spec = spec or {}
    ext = spec.get("extrinsic", args.extrinsic)
    lengths = args.rpe_lengths
    return Options(
        t_offset=float(spec.get("t_offset", args.t_offset)),
        extrinsic=lio.parse_extrinsic(ext) if ext else None,
        max_gap=args.max_gap, align=args.align, head_m=args.head_m,
        rpe_lengths=[float(x) for x in lengths.split(",")] if lengths else None,
        jump_m=args.jump_m, jump_deg=args.jump_deg, fail_abs=args.fail_abs, fail_rel=args.fail_rel,
        min_coverage=args.min_coverage, success_ate=args.success_ate,
        closed_loop=bool(spec.get("closed_loop", args.closed_loop)),
        loop_radius=args.loop_radius, loop_trans=args.loop_trans, loop_rot=args.loop_rot,
        loop_min_dt=args.loop_min_dt)


def cmd_run(args):
    from .evaluate import evaluate_run
    from .report import make_figures, markdown_report, setup_fonts, terminal_summary, write_json
    run = lio.Run(args.run)
    gt = lio.load_trajectory(args.gt, args.gt_format) if args.gt else None
    summary, series = evaluate_run(run, gt, _options(args))
    out = args.out or os.path.join(args.run, "eval")
    os.makedirs(out, exist_ok=True)
    figs = []
    if not args.no_plots:
        setup_fonts()
        figs = make_figures(summary, series, run, os.path.join(out, "figs"))
    write_json(os.path.join(out, "summary.json"), summary)
    with open(os.path.join(out, "report.md"), "w") as f:
        f.write(markdown_report(summary, figs))
    print(terminal_summary(summary))
    print("\n报告: %s" % os.path.join(out, "report.md"))
    return 0 if summary["verdict"]["success"] else 2


def cmd_batch(args):
    from .batch import load_manifest, print_batch, run_batch
    manifest = load_manifest(args.manifest) if args.manifest else None
    result = run_batch(args.root, lambda spec: _options(args, spec), gt_dir=args.gt_dir,
                       manifest=manifest, out_dir=args.out, plots=not args.no_plots)
    print()
    print_batch(result)
    print("\n报告: %s" % os.path.join(args.out or os.path.join(args.root, "eval"), "batch_report.md"))
    return 0 if result["overall"]["success_rate"] == 1.0 else 2


def cmd_compare(args):
    from .batch import compare
    from .report import md_table, text_table
    rows, regressions = compare(args.base, args.new, args.threshold)
    print(text_table(rows))
    if args.out:
        with open(args.out, "w") as f:
            f.write("# 对比：%s → %s\n\n阈值 %.1f%%，且超过重复运行标准差之和才标记。\n\n%s\n" % (
                args.base, args.new, args.threshold, md_table(rows)))
    if regressions:
        print("\n%d 项显著退步" % len(regressions))
        return 3 if args.fail_on_regression else 0
    print("\n无显著退步")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(prog="lvi_eval", description="lvi_slam 精度 / 稳定性 / 效率评估")
    p.add_argument("--version", action="version", version="lvi_eval " + __version__)
    sub = p.add_subparsers(dest="cmd")

    r = sub.add_parser("run", help="评估一次运行（lvi_offline --out-dir 的输出）")
    r.add_argument("--run", required=True, help="运行目录")
    r.add_argument("--gt", default=None, help="真值轨迹；不给则只算无真值指标")
    r.add_argument("--out", default=None, help="输出目录（默认 <run>/eval）")
    _add_eval_options(r)
    r.set_defaults(func=cmd_run)

    b = sub.add_parser("batch", help="评估一个目录下的多个序列（及重复运行）")
    b.add_argument("root", help="根目录：每个子目录是一个序列的运行目录，或包含多次重复运行")
    b.add_argument("--gt-dir", default=None, help="真值目录：<序列名>.tum/.txt/.csv")
    b.add_argument("--manifest", default=None, help="JSON/YAML：逐序列指定 gt、t_offset、extrinsic")
    b.add_argument("--out", default=None, help="输出目录（默认 <root>/eval）")
    _add_eval_options(b)
    b.set_defaults(func=cmd_batch)

    c = sub.add_parser("compare", help="对比两次评测（batch 目录或单次运行目录）")
    c.add_argument("base")
    c.add_argument("new")
    c.add_argument("--threshold", type=float, default=5.0, help="显著变化阈值 (%%)")
    c.add_argument("--out", default=None, help="同时写一份 Markdown")
    c.add_argument("--fail-on-regression", action="store_true", help="有显著退步时返回非零（CI 用）")
    c.set_defaults(func=cmd_compare)

    args = p.parse_args(argv)
    if not getattr(args, "func", None):
        p.print_help()
        return 1
    try:
        return args.func(args)
    except lio.EvalError as e:
        print("错误: %s" % e, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
