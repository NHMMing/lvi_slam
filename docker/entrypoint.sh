#!/usr/bin/env bash
# 容器入口：加载 ROS 与工作空间环境，把 lvi_offline / lvi_eval 放进 PATH。
# 在线节点照常用 roslaunch / ros2 launch。
set -e
WS="${WS:-/ws}"
SRC="${SRC:-/lvi_slam}"

# HOME 可能不可写（以宿主机 UID 运行时容器里没有这个用户）
if [[ ! -w "${HOME:-/}" ]]; then export HOME=/tmp/home; mkdir -p "$HOME"; fi
export ROS_LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$HOME/.config/matplotlib}"

# ROS 的 setup.bash 引用未定义变量，这里不开 -u
case "${ROS_DISTRO:-none}" in
  noetic|humble) source "/opt/ros/$ROS_DISTRO/setup.bash" ;;
esac
if [[ -f "$WS/install/setup.bash" ]]; then
  source "$WS/install/setup.bash"
fi

# lvi_offline 的安装位置因构建方式而异，统一加进 PATH
for d in "$WS/install/lib/lvi_slam" "$WS/install/bin"; do
  [[ -x "$d/lvi_offline" ]] && export PATH="$d:$PATH"
done
# 评估工具：优先用挂载进来的源码（开发时改了立即生效）
if [[ -d "$SRC/tools/eval" ]]; then
  export PATH="$SRC/tools/eval/bin:$PATH"
fi
export LD_LIBRARY_PATH="$WS/install/lib:${LD_LIBRARY_PATH:-}"

exec "$@"
