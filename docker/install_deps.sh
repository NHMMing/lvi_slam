#!/usr/bin/env bash
# 安装 lvi_slam 的全部编译/运行依赖。Dockerfile 用它构建 deps 镜像；
# 也可以直接在裸机上跑（例如 Orin 不想用容器时）：
#   sudo ROS_DISTRO=noetic docker/install_deps.sh     # ROS1（需已配置 ROS apt 源）
#   sudo ROS_DISTRO=humble docker/install_deps.sh     # ROS2
#   sudo ROS_DISTRO=none   docker/install_deps.sh     # 无 ROS
#
# 版本策略：Eigen / PCL / OpenCV / yaml-cpp / Boost 一律用发行版 apt 版本——
# ROS 的二进制包（cv_bridge、pcl_conversions）就是用它们编的，换了必出 ABI 问题。
# 唯一源码编译的是 GTSAM（20.04/22.04 都没有合适的 apt 包），编译选项必须与
# CMakeLists.txt 的校验一致：系统 Eigen、不开 -march=native。
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-none}"
GTSAM_VERSION="${GTSAM_VERSION:-4.2}"
GTSAM_PREFIX="${GTSAM_PREFIX:-/opt/gtsam}"
JOBS="${JOBS:-$(nproc)}"
export DEBIAN_FRONTEND=noninteractive

log() { echo -e "\033[1;32m[deps]\033[0m $*"; }

common=(
  build-essential cmake git pkg-config ca-certificates curl
  libeigen3-dev libpcl-dev libopencv-dev libyaml-cpp-dev
  libboost-all-dev libtbb-dev
  libgtest-dev
  # 评估工具 tools/eval 的依赖（用 apt 版本，避免 pip 与系统 numpy 混装）
  python3 python3-numpy python3-matplotlib python3-yaml fonts-wqy-microhei
)

case "$ROS_DISTRO" in
  noetic)
    ros=(
      python3-catkin-tools
      ros-noetic-catkin ros-noetic-roscpp ros-noetic-rosbag
      ros-noetic-sensor-msgs ros-noetic-nav-msgs ros-noetic-geometry-msgs ros-noetic-std-msgs
      ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-tf2-eigen
      ros-noetic-cv-bridge ros-noetic-pcl-conversions
      ros-noetic-message-generation ros-noetic-message-runtime
    ) ;;
  humble)
    ros=(
      python3-colcon-common-extensions
      ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs
      ros-humble-std-msgs ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-eigen
      ros-humble-cv-bridge ros-humble-pcl-conversions
      ros-humble-rosbag2-cpp ros-humble-rosbag2-storage
      ros-humble-rosbag2-storage-default-plugins ros-humble-rosbag2-storage-mcap
      ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime
      ros-humble-ament-cmake-gtest
    ) ;;
  none) ros=() ;;
  *) echo "不支持的 ROS_DISTRO: $ROS_DISTRO（noetic | humble | none）" >&2; exit 1 ;;
esac

log "apt 依赖（ROS_DISTRO=$ROS_DISTRO）"
apt-get update
apt-get install -y --no-install-recommends "${common[@]}" "${ros[@]}"

if [[ -f "$GTSAM_PREFIX/lib/cmake/GTSAM/GTSAMConfig.cmake" ]]; then
  log "GTSAM 已存在于 $GTSAM_PREFIX，跳过"
else
  log "源码编译 GTSAM $GTSAM_VERSION → $GTSAM_PREFIX"
  src=$(mktemp -d)
  git clone --depth 1 --branch "$GTSAM_VERSION" https://github.com/borglab/gtsam.git "$src"
  cmake -S "$src" -B "$src/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$GTSAM_PREFIX" \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_UNSTABLE=OFF \
    -DGTSAM_BUILD_PYTHON=OFF \
    -DGTSAM_WITH_TBB=ON
  cmake --build "$src/build" -j"$JOBS"
  cmake --install "$src/build"
  rm -rf "$src"
  # 让动态链接器找到 libgtsam.so，容器内和裸机都不用再设 LD_LIBRARY_PATH
  echo "$GTSAM_PREFIX/lib" > /etc/ld.so.conf.d/gtsam.conf
  ldconfig
fi

# 镜像里清掉 apt 索引减小体积（Dockerfile 设 CLEAN_APT=1）；裸机上保留。
# 不用 /.dockerenv 判断：BuildKit 构建期间没有这个文件
[[ "${CLEAN_APT:-0}" == 1 ]] && rm -rf /var/lib/apt/lists/*
log "完成"
