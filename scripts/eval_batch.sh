#!/usr/bin/env bash
# 批量评测脚本。
#
# 单序列跑一次的数字没有意义 —— 参数调好可能只是过拟合了那一个序列。
# 必须在整个数据集上跑，看平均值和最差情况。
set -euo pipefail

CONFIG=${1:?用法: eval_batch.sh <config.yaml> <数据根目录> <真值目录> [profile]}
DATA_ROOT=${2:?}
GT_ROOT=${3:?}
PROFILE=${4:-livox}

OUT_DIR="results/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT_DIR"
cp "$CONFIG" "$OUT_DIR/config_used.yaml"   # 记录当次参数，否则事后无法复现

SUMMARY="$OUT_DIR/summary.csv"
echo "sequence,ate_rmse,ate_mean,ate_max,rpe_rmse,realtime_factor" > "$SUMMARY"

for seq_dir in "$DATA_ROOT"/*/; do
  seq=$(basename "$seq_dir")
  gt="$GT_ROOT/${seq}.tum"
  [[ -f "$gt" ]] || { echo "跳过 $seq（无真值）"; continue; }

  echo "===== $seq ====="
  traj="$OUT_DIR/${seq}.tum"
  log="$OUT_DIR/${seq}.log"

  lvi_offline --config "$CONFIG" --data "$seq_dir" --out "$traj" \
              --profile "$PROFILE" 2>&1 | tee "$log"

  rt=$(grep -oP '实时倍率: \K[0-9.]+' "$log" || echo "0")

  # -as: 对齐并估计尺度（单目才需要，LiDAR 系统用 -a 即可）
  ate=$(evo_ape tum "$gt" "$traj" -a 2>/dev/null | \
        grep -E "rmse|mean|max" | awk '{print $2}' | paste -sd,)
  rpe=$(evo_rpe tum "$gt" "$traj" -a --delta 100 --delta_unit m 2>/dev/null | \
        grep "rmse" | awk '{print $2}')

  echo "$seq,$ate,$rpe,$rt" >> "$SUMMARY"
done

echo ""
echo "===== 汇总 ====="
column -t -s, "$SUMMARY"
echo ""
echo "结果目录: $OUT_DIR"

# 关键：把平均 ATE 打出来，这是唯一该盯的单一数字
awk -F, 'NR>1 && $2!="" {s+=$2; n++} END {if(n>0) printf "平均 ATE RMSE: %.4f m (%d 个序列)\n", s/n, n}' "$SUMMARY"
