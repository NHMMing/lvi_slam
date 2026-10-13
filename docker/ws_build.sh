#!/usr/bin/env bash
# 容器内编译 lvi_slam（Dockerfile 的 build 阶段与 docker/lvi.sh build 共用）。
# 按 ROS_DISTRO 选择构建方式，产物统一装到 $WS/install：
#   noetic → catkin_make（install 空间）
#   humble → colcon
#   none   → 纯 CMake
#
# 环境变量:
#   SRC         仓库路径          默认 /lvi_slam
#   WS          工作空间          默认 /ws
#   BUILD_TYPE  Release | Debug  默认 Release
#   RUN_TESTS   1 = 编译后跑单元测试（默认 1）
#   JOBS        并行数            默认 nproc
set -euo pipefail

SRC="${SRC:-/lvi_slam}"
WS="${WS:-/ws}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
RUN_TESTS="${RUN_TESTS:-1}"
JOBS="${JOBS:-$(nproc)}"
GTSAM_DIR="${GTSAM_DIR:-/opt/gtsam/lib/cmake/GTSAM}"
ROS_DISTRO="${ROS_DISTRO:-none}"

log() { echo -e "\033[1;32m[build]\033[0m $*"; }

# 构建类型变化时清缓存：CMakeCache 会记住旧值
stamp="$WS/.build_type"
if [[ -f "$stamp" && "$(cat "$stamp")" != "$BUILD_TYPE" ]]; then
  log "构建类型变了（$(cat "$stamp") → $BUILD_TYPE），清理后全量编译"
  rm -rf "$WS/build" "$WS/devel" "$WS/install" "$WS/log"
fi
mkdir -p "$WS"
echo "$BUILD_TYPE" > "$stamp"

common_args=(-DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DNATIVE_ARCH=OFF -DGTSAM_DIR="$GTSAM_DIR")

case "$ROS_DISTRO" in
  noetic)
    set +u; source /opt/ros/noetic/setup.bash; set -u
    # catkin 不进入 lvi_slam 包内部找子包，消息包要单独放进 src/
    mkdir -p "$WS/src"
    for p in "$SRC" "$SRC/third_party/livox_ros_driver" "$SRC/third_party/livox_ros_driver2"; do
      ln -sfn "$p" "$WS/src/$(basename "$p")"
    done
    cd "$WS"
    log "catkin_make install（$BUILD_TYPE）"
    catkin_make install "${common_args[@]}" -DCATKIN_ENABLE_TESTING="$RUN_TESTS" -j"$JOBS"
    if [[ "$RUN_TESTS" == 1 ]]; then
      catkin_make tests -j"$JOBS" >/dev/null
      log "单元测试"
      "$WS/devel/lib/lvi_slam/test_core"
    fi
    ;;
  humble)
    set +u; source /opt/ros/humble/setup.bash; set -u
    cd "$WS"
    log "colcon build（$BUILD_TYPE）"
    # 显式列出 base-paths：colcon 发现一个包后不再进入它的子目录找包
    colcon build \
      --base-paths "$SRC" "$SRC/third_party/livox_ros_driver2" \
      --build-base "$WS/build" --install-base "$WS/install" \
      --event-handlers console_cohesion+ \
      --parallel-workers "$JOBS" \
      --cmake-args "${common_args[@]}" -DBUILD_TESTING="$RUN_TESTS"
    if [[ "$RUN_TESTS" == 1 ]]; then
      log "单元测试"
      "$WS/build/lvi_slam/test_core"
    fi
    ;;
  none)
    log "CMake（无 ROS，$BUILD_TYPE）"
    cmake -S "$SRC" -B "$WS/build" "${common_args[@]}" -DLVI_ROS_VERSION=0 \
          -DBUILD_TESTING="$RUN_TESTS" -DCMAKE_INSTALL_PREFIX="$WS/install"
    cmake --build "$WS/build" -j"$JOBS"
    cmake --install "$WS/build"
    if [[ "$RUN_TESTS" == 1 ]]; then
      log "单元测试"
      "$WS/build/test_core"
    fi
    ;;
  *) echo "不支持的 ROS_DISTRO: $ROS_DISTRO" >&2; exit 1 ;;
esac
log "完成，产物在 $WS/install"
