#!/usr/bin/env bash
# ROS1 (catkin) 一键编译脚本。
#
# 编译选项和环境坑都收在这里，不用记：
#   - conda 的 python 没有 catkin_pkg，且会把 conda 的 yaml-cpp / gtest 混进来 → 从环境中剔除
#   - GTSAM 必须用不带 -march=native 的那份（与 apt 的 PCL 对齐一致，见 CMakeLists.txt）
#   - 换了 GTSAM 或构建类型后，catkin 的 CMake 缓存会记住旧值 → 检测到变化自动清理重编
#
# 用法:
#   scripts/build_ros1.sh            # 增量编译（Release）
#   scripts/build_ros1.sh --clean    # 删除 build/devel 后全量编译
#   scripts/build_ros1.sh --debug    # Debug 构建（会自动全量重编）
#   scripts/build_ros1.sh -j 4       # 指定并行数（默认 nproc）
#
# 可用环境变量覆盖默认值:
#   LVI_WS      catkin 工作空间        默认 ~/lvi_ws
#   GTSAM_DIR   GTSAMConfig.cmake 目录 默认 ~/library/gtsam/install_nonative/lib/cmake/GTSAM
#   ROS_DISTRO  ROS1 发行版            默认 noetic
set -euo pipefail

usage() { sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; }

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LVI_WS="${LVI_WS:-$HOME/lvi_ws}"
GTSAM_DIR="${GTSAM_DIR:-$HOME/library/gtsam/install_nonative/lib/cmake/GTSAM}"
ROS_DISTRO="${ROS_DISTRO:-noetic}"
BUILD_TYPE=Release
JOBS="$(nproc)"
CLEAN=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -c|--clean) CLEAN=1 ;;
    -d|--debug) BUILD_TYPE=Debug ;;
    -j)         JOBS="${2:?-j 需要一个数字}"; shift ;;
    -j*)        JOBS="${1#-j}" ;;
    -h|--help)  usage; exit 0 ;;
    *)          echo "未知参数: $1" >&2; usage; exit 1 ;;
  esac
  shift
done

# ---- 1. 剔除 conda ----
# 只改本脚本进程的环境，不影响调用者的 shell。
if [[ -n "${CONDA_PREFIX:-}" || "$PATH" == *conda* ]]; then
  PATH="$(tr ':' '\n' <<<"$PATH" | grep -v -i conda | paste -sd: -)"
  unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE CONDA_SHLVL PYTHONPATH
  export PATH
  echo "[build] 已从环境中剔除 conda"
fi

# ---- 2. 检查依赖 ----
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
[[ -f "$ROS_SETUP" ]] || { echo "[build] 找不到 $ROS_SETUP" >&2; exit 1; }
[[ -f "$GTSAM_DIR/GTSAMConfig.cmake" ]] || {
  echo "[build] 找不到 $GTSAM_DIR/GTSAMConfig.cmake" >&2
  echo "        GTSAM 需用 -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF 编译，或用 GTSAM_DIR=... 指定" >&2
  exit 1
}

# ROS 的 setup.bash 会引用未定义变量，临时关掉 -u
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

# ---- 3. 准备工作空间（缺什么补什么，已存在的不动）----
mkdir -p "$LVI_WS/src"
link_pkg() {   # link_pkg <目标> <链接名>
  local target="$1" link="$LVI_WS/src/$2"
  if [[ -L "$link" ]]; then
    [[ "$(readlink -f "$link")" == "$(readlink -f "$target")" ]] ||
      echo "[build] 警告: $link 指向 $(readlink -f "$link")，不是当前仓库" >&2
  elif [[ -e "$link" ]]; then
    echo "[build] 警告: $link 已存在且不是软链接，保持不动" >&2
  else
    ln -s "$target" "$link"
    echo "[build] 创建软链接 $link -> $target"
  fi
}
link_pkg "$REPO_DIR" lvi_slam
link_pkg "$REPO_DIR/third_party/livox_ros_driver" livox_ros_driver   # 仅消息定义，回放 Livox bag 用

# ---- 4. 缓存与本次参数不一致时自动全量重编 ----
CACHE="$LVI_WS/build/CMakeCache.txt"
if [[ $CLEAN -eq 0 && -f "$CACHE" ]]; then
  cached_gtsam="$(sed -n 's/^GTSAM_DIR:[A-Z]*=//p' "$CACHE")"
  cached_type="$(sed -n 's/^CMAKE_BUILD_TYPE:[A-Z]*=//p' "$CACHE")"
  if [[ "$cached_gtsam" != "$GTSAM_DIR" ]]; then
    echo "[build] GTSAM_DIR 变了（$cached_gtsam → $GTSAM_DIR），全量重编"
    CLEAN=1
  elif [[ "$cached_type" != "$BUILD_TYPE" ]]; then
    echo "[build] 构建类型变了（$cached_type → $BUILD_TYPE），全量重编"
    CLEAN=1
  fi
fi
if [[ $CLEAN -eq 1 ]]; then
  echo "[build] 删除 $LVI_WS/build $LVI_WS/devel"
  rm -rf "$LVI_WS/build" "$LVI_WS/devel"
fi

# ---- 5. 编译 ----
echo "[build] 工作空间 $LVI_WS  类型 $BUILD_TYPE  并行 $JOBS"
echo "[build] GTSAM $GTSAM_DIR"
cd "$LVI_WS"
catkin_make \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DNATIVE_ARCH=OFF \
  -DGTSAM_DIR="$GTSAM_DIR" \
  -j"$JOBS"

echo
echo "[build] 完成。使用前:  source $LVI_WS/devel/setup.zsh   (bash 用 setup.bash)"
echo "        离线回放:      rosrun lvi_slam lvi_offline --config <yaml> --data <bag>"
echo "        在线运行:      roslaunch lvi_slam lvi_slam.launch config_file:=<yaml>"
