#!/usr/bin/env bash
# 批量评测：对数据根目录下每个序列跑 lvi_offline（可重复 N 次），再用 lvi_eval batch 汇总。
#
# 单序列跑一次的数字没有意义 —— 参数调好可能只是过拟合了那一个序列。
# 必须在整个数据集上跑，看平均值和最差情况；重复运行看离散度（多线程时序的不确定性）。
#
# 用法:
#   scripts/eval_batch.sh -c <config.yaml> -d <数据根目录> [选项] [-- lvi_offline 额外参数...]
#
#   -c, --config <yaml>     算法配置（必需）
#   -d, --data <dir>        数据根目录（必需）：每个条目是一个序列（.bag 文件、ROS2 bag 目录或 KITTI 目录）
#   -g, --gt-dir <dir>      真值目录：<序列名>.tum/.txt/.csv；不给则只算无真值指标
#   -m, --manifest <file>   逐序列的真值/时间偏移/外参（见 lvi_eval batch --help），与 -g 二选一
#   -p, --profile <name>    点云剖面，默认 livox
#   -n, --repeat <N>        每个序列重复次数，默认 1（看稳定性建议 ≥3）
#   -o, --out <dir>         结果目录，默认 results/<时间戳>
#   -b, --baseline <dir>    与之前的结果目录对比，有显著退步时退出码非零
#   --eval-args "<...>"     传给 lvi_eval batch 的额外参数，如 "--closed-loop --align se3"
#
# 例:
#   scripts/eval_batch.sh -c config/livox_avia.yaml -d ~/datasets/mars_lvig -g ~/datasets/mars_lvig/gt -n 3
#   scripts/eval_batch.sh -c config/livox_avia.yaml -d ~/datasets/mars_lvig -g ~/datasets/mars_lvig/gt \
#       -b results/20261013_120000 -- --imu-topic /livox/imu --lidar-topic /livox/lidar
set -euo pipefail

usage() { sed -n '2,26p' "$0" | sed 's/^# \{0,1\}//'; }

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="" DATA="" GT_DIR="" MANIFEST="" PROFILE=livox REPEAT=1 OUT="" BASELINE="" EVAL_ARGS=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    -c|--config)   CONFIG="$2"; shift ;;
    -d|--data)     DATA="$2"; shift ;;
    -g|--gt-dir)   GT_DIR="$2"; shift ;;
    -m|--manifest) MANIFEST="$2"; shift ;;
    -p|--profile)  PROFILE="$2"; shift ;;
    -n|--repeat)   REPEAT="$2"; shift ;;
    -o|--out)      OUT="$2"; shift ;;
    -b|--baseline) BASELINE="$2"; shift ;;
    --eval-args)   EVAL_ARGS="$2"; shift ;;
    -h|--help)     usage; exit 0 ;;
    --)            shift; break ;;
    *) echo "未知参数: $1" >&2; usage; exit 1 ;;
  esac
  shift
done
[[ -n "$CONFIG" && -n "$DATA" ]] || { usage; exit 1; }
EXTRA=("$@")

command -v lvi_offline >/dev/null || { echo "找不到 lvi_offline：先 source 工作空间，或在容器里运行（docker/lvi.sh run）" >&2; exit 1; }
LVI_EVAL="$(command -v lvi_eval || echo "$REPO/tools/eval/bin/lvi_eval")"

OUT="${OUT:-results/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"
echo "结果目录: $OUT"

shopt -s nullglob
n_fail=0
for entry in "$DATA"/*; do
  name="$(basename "$entry")"
  [[ "$name" == gt || "$name" == groundtruth ]] && continue
  if [[ -f "$entry" ]]; then
    [[ "$name" == *.bag || "$name" == *.mcap || "$name" == *.db3 ]] || continue
    seq="${name%.*}"
  else
    seq="$name"
  fi

  for ((k = 1; k <= REPEAT; k++)); do
    run_dir="$OUT/$seq"
    (( REPEAT > 1 )) && run_dir="$OUT/$seq/run$k"
    echo "===== $seq ($k/$REPEAT) ====="
    mkdir -p "$run_dir"
    rc=0
    lvi_offline --config "$CONFIG" --data "$entry" --profile "$PROFILE" \
                --out-dir "$run_dir" "${EXTRA[@]}" > "$run_dir/console.log" 2>&1 || rc=$?
    if (( rc != 0 )); then
      # 崩溃本身就是稳定性结果：写一个占位 run_meta.json，lvi_eval 会把这次运行按失败计入，
      # 而不是因为目录里没有产物就悄悄跳过、虚高成功率
      echo "  lvi_offline 异常退出（$rc），见 $run_dir/console.log"
      [[ -f "$run_dir/run_meta.json" ]] ||
        printf '{"crashed": true, "exit_code": %d}\n' "$rc" > "$run_dir/run_meta.json"
      n_fail=$((n_fail + 1))
    fi
  done
done

eval_args=(batch "$OUT")
[[ -n "$GT_DIR" ]] && eval_args+=(--gt-dir "$GT_DIR")
[[ -n "$MANIFEST" ]] && eval_args+=(--manifest "$MANIFEST")
# shellcheck disable=SC2206
[[ -n "$EVAL_ARGS" ]] && eval_args+=($EVAL_ARGS)
"$LVI_EVAL" "${eval_args[@]}"

(( n_fail > 0 )) && echo "注意: $n_fail 次运行异常退出"
if [[ -n "$BASELINE" ]]; then
  "$LVI_EVAL" compare "$BASELINE" "$OUT" --out "$OUT/eval/compare.md" --fail-on-regression
fi
