# LVI-SLAM

紧耦合 LiDAR-Visual-Inertial SLAM。**前端 iEKF + 后端因子图**混合架构，针对 Livox 非重复扫描雷达优化。同时支持 ROS1 (Noetic) 和 ROS2 (Humble/Jazzy)。

## 为什么再造一个轮子

三个主流系统各缺一块：

| | 缺什么 |
|---|---|
| FAST-LIVO2 | 没有回环，长距离必漂移 |
| LIO-SAM | 没有视觉；假设硬件时间同步；权重靠手工调 |
| VINS-Mono | 没有激光；单目尺度靠运动激励 |

本系统的目标是三者取长：FAST-LIVO2 级别的局部精度 + LIO-SAM 级别的全局一致性 + VINS-Mono 级别的时间同步鲁棒性。

## 架构

```
Livox ─┐
IMU  ──┼─→ DataSynchronizer ──→ Frontend (iEKF) ──→ Backend (GTSAM 因子图)
Cam  ──┘   · 按图像时刻切分点云    · LIO/VIO 同时刻交替     · Scan Context 回环
           · td 在线估计           · 统一体素地图            · 双模态验证
                                   · 局部滑窗，内存有界       · GPS 因子（可选）
                                            ▲                      │
                                            └── applyLoopCorrection ┘
```

### 四个关键设计

**1. 前端滤波，后端因子图**

滤波用对偶形式卡尔曼增益 `K = (HᵀR⁻¹H + P⁻¹)⁻¹HᵀR⁻¹`，求逆矩阵只有 26×26，**计算量与观测数解耦** —— 这是整帧点云 + 上万维光度残差能实时跑的唯一原因。

但回环在滤波框架下无处安放（EKF 是马尔可夫的，回不去改历史），所以全局一致性交给因子图。

**2. 地图 = 关键帧位姿 + 关键帧 body 系点云**

前端体素地图只是局部工作地图，带滑窗、内存有界。全局地图由后端关键帧隐式定义，`buildGlobalMap()` 按需拼接。

回环改了位姿，全局地图自动就对。这避开了「FAST-LIO2 + 外接位姿图」的结构性问题 —— 它的 ikd-Tree 存的是已变换到世界系的点，位姿改了地图改不动。

**3. 权重全部从噪声模型推导，零手工调参**

```
calcPointCov()  逐点椭球协方差（深度误差 + 角度误差×距离）
      ↓
plane_var       平面参数自身的 6×6 协方差（点越多越小）
      ↓
R_inv = 1/(ε + σ_plane + nᵀ·var·n)
```

对比 LIO-SAM 的 `s = 1 - 0.9*fabs(dist)`（手工线性衰减）。卡方检验顺带完成外点剔除，不需要额外鲁棒核。换传感器只改 yaml，不改代码。

**4. 混合视觉前端**

直接法怕光照突变，特征点法怕弱纹理，失效场景几乎不重叠。用「本帧光度残差中位数 / 历史中位数」的比值做切换判据（不是绝对阈值 —— 绝对值在不同场景差异太大），带迟滞防抖。

特征点轨即使正常时也低频运行 —— 直接法没有描述子，做不了视觉回环。

## 目录

```
include/lvi_slam/
  common/     types.h(状态定义) so3_math.h config.h
  preprocess/ livox_preprocess.h    ← Livox tag 滤波 + 时间偏移
  sync/       data_synchronizer.h   ← LIO/VIO 交替调度，点云切分
  imu/        imu_processor.h       ← 传播 + 去畸变（旋转&平移都补）
  map/        voxel_map.h           ← 自适应八叉树，几何+视觉共用索引
  estimator/  iekf.h                ← 对偶形式增益 + 退化分析
  lio/        lio_updater.h         ← 点-面残差
  vio/        vio_updater.h         ← 直接法 + 特征法 + 调度器
  frontend/   frontend.h
  backend/    backend.h loop_detector.h
  system.h
```

## 编译

同一份源码支持 ROS1 和 ROS2，由环境变量 `ROS_VERSION`（source 对应的 setup 后自动设置）决定；
也可用 `-DLVI_ROS_VERSION=1|2|0` 强制指定（0 = 无 ROS，只编核心库和离线工具）。

核心算法（`src/`、`include/`）完全不依赖 ROS，两个版本只在下面三处不同：

| | ROS1 (Noetic) | ROS2 (Humble+) |
|---|---|---|
| 节点 | `ros1/lvi_slam_node.cpp` | `ros2/lvi_slam_node.cpp` |
| bag 读取 | `src/io/rosbag1_source.cpp`（`.bag`） | `src/io/ros2bag_source.cpp`（`.db3`/`.mcap`） |
| launch | `launch/lvi_slam.launch` | `launch/lvi_slam.launch.py` |

```bash
# 公共依赖
sudo apt install libpcl-dev libeigen3-dev libopencv-dev libyaml-cpp-dev
# GTSAM 4.2 源码编译（Ubuntu 20.04 没有 apt 包）。Eigen 和 -march=native 必须与本工程一致，
# CMake 配置阶段会检查，不一致直接报错。本工程默认不开 -march=native（见 CMakeLists.txt），
# 所以 GTSAM 也要关掉；Eigen 显式指到系统的 3.3.7，否则可能误用 /usr/local 下的其他版本：
#   cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON \
#     -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 -DEigen3_DIR=/usr/lib/cmake/eigen3 \
#     -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
#     -DCMAKE_INSTALL_PREFIX=~/library/gtsam/install_nonative -DCMAKE_INSTALL_RPATH='$ORIGIN'
# 装到非系统目录、不跑 ldconfig 时，CMAKE_INSTALL_RPATH='$ORIGIN' 让 libgtsam 找到同目录的
# libmetis-gtsam.so。
# 依赖版本由 LVI_{EIGEN,PCL,OPENCV,YAML_CPP,GTSAM}_VERSION 锁定，默认是 Noetic 的 apt 版本
# （Eigen 3.3.7 / PCL 1.10 / OpenCV 4.2 / yaml-cpp 0.6.2 / GTSAM 4.2）。配置前先 conda deactivate。

# ---- ROS1 Noetic (Ubuntu 20.04) ----
sudo apt install ros-noetic-desktop
# 需要 livox_ros_driver 放进同一 catkin workspace。回放 bag 用仓库自带的
# third_party/livox_ros_driver 即可（只有消息定义，不需要 Livox-SDK）；
# catkin 不会进入 lvi_slam 包内部查找，所以要单独软链接到 src/。
# 接真实雷达时换成官方完整驱动；用 Mid-360 驱动时加 -DLVI_LIVOX_DRIVER=livox_ros_driver2
#
# 一键编译脚本：自动剔除 conda、创建上面两个软链接、带上全部编译选项，
# GTSAM 路径或构建类型变化时自动全量重编。选项见 scripts/build_ros1.sh --help
scripts/build_ros1.sh                 # 默认工作空间 ~/lvi_ws，Release
scripts/build_ros1.sh --clean         # 全量重编
LVI_WS=~/other_ws GTSAM_DIR=/path/to/gtsam/lib/cmake/GTSAM scripts/build_ros1.sh

# ---- ROS2 Humble (Ubuntu 22.04) ----
sudo apt install ros-humble-desktop
# 回放 bag 用仓库自带的 third_party/livox_ros_driver2（只有消息定义，不需要 Livox-SDK2）；
# colcon 发现 lvi_slam 后不进入其子目录，所以要显式列出 base-paths
cd ~/ros2_ws && colcon build \
  --base-paths src/lvi_slam src/lvi_slam/third_party/livox_ros_driver2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`-march=native` 默认关闭（`NATIVE_ARCH=OFF`）：apt 的 PCL/OpenCV 按 16 字节对齐编译，开启后 Eigen 对齐不一致会在内存释放时崩溃。只有所有 Eigen 依赖都用 `-march=native` 自编译时才可以 `-DNATIVE_ARCH=ON`。

## Docker（推荐）

宿主机只需要 Docker，其余依赖（含源码编译的 GTSAM）全在镜像里，换机器不用再配环境。
一个 `docker/Dockerfile` 覆盖三个变体 × 两种架构：

| 变体 | 基础镜像 | 用途 |
|---|---|---|
| `noetic` | Ubuntu 20.04 + ROS1 Noetic | 默认；直接读 `.bag` |
| `humble` | Ubuntu 22.04 + ROS2 Humble | ROS2 节点 / `.db3` `.mcap` |
| `noros` | Ubuntu 22.04 | 只有核心库 + `lvi_offline` + `lvi_eval` |

架构：`linux/amd64` 与 `linux/arm64`（Jetson Orin）。GitHub Actions（`.github/workflows/docker.yml`）
在每次 push 时编译全部 6 个组合并跑单元测试，推送到 `ghcr.io/nhmming/lvi_slam:<变体>[-deps]`。

```bash
# 1. 新机器一次性准备（安装 Docker；有 NVIDIA 显卡时顺带装容器运行时，用于 RViz）
sudo -E docker/setup_host.sh            # 之后重新登录一次，让 docker 组生效
#    Jetson：JetPack 自带 Docker，只需 sudo usermod -aG docker $USER

# 2. 获取依赖镜像：优先从 GHCR 拉，拉不到则本地构建（首次 20-40 分钟，主要是编译 GTSAM）
docker/lvi.sh pull noetic
docker/lvi.sh image noetic              # 本地构建；国内可加 APT_MIRROR=http://mirrors.tuna.tsinghua.edu.cn

# 3. 开发循环：源码挂载进容器，产物在 ~/.cache/lvi_slam/<变体>，不污染仓库
docker/lvi.sh build noetic              # 增量编译 + 单元测试（--debug / --no-test）
docker/lvi.sh shell noetic              # 交互式进入

# 4. 跑数据 + 评估。目录映射：~/datasets → /data，<仓库>/results → /results
docker/lvi.sh run noetic -- lvi_offline --config /lvi_slam/config/livox_avia.yaml \
    --data /data/mars_lvig/HKisland01.bag --out-dir /results/hk01
docker/lvi.sh eval noetic run --run /results/hk01 --gt /data/mars_lvig/gt/HKisland01.tum

# Orin 镜像：本机交叉构建（QEMU，慢）或直接在 Orin 上 docker/lvi.sh image
sudo -E docker/setup_host.sh --arm64
docker/lvi.sh image noetic --platform linux/arm64 --push
```

不想用容器的机器（例如 Orin 裸机）可以直接跑同一份依赖脚本：`sudo ROS_DISTRO=noetic docker/install_deps.sh`。

## 评估

自研评估工具 `tools/eval`（`lvi_eval`，只依赖 numpy，画图用 matplotlib），读 `lvi_offline --out-dir` 的输出：

| 文件 | 内容 |
|---|---|
| `odometry.tum` | 前端逐帧在线位姿（回环发生时会跳变） |
| `slam.tum` | 同一组帧，按后端最终优化结果重新表达 |
| `keyframes.tum` | 后端关键帧最终位姿 |
| `frames.csv` | 逐帧诊断：耗时、残差、退化、协方差、bias、td |
| `loops.csv` | 回环候选及每一级验证的结果 |
| `resources.csv` / `run_meta.json` | 内存/CPU 采样；版本、参数、统计 |

**odometry 与 slam 分开输出，是为了能量化后端和回环到底起了多大作用**：同一次运行里两条轨迹的 ATE /
漂移之差就是后端收益；`loops.csv` 结合真值给出回环的精度（有没有错误回环）和召回（该闭的环闭上了没有）。

指标分四组：

- **精度**：ATE（默认 4 自由度对齐，可选 SE3/Sim3）、只用开头 20m 对齐的漂移误差、按路程的 RPE（%，°/100m）、按时间的 RPE
- **稳定性**：输出覆盖率、断流、跳变、发散检测、同一序列重复运行的离散度、成功率
- **一致性**：NEES（滤波器协方差是否可信）
- **效率**：单帧耗时分位数、实时倍率、峰值内存、缓冲溢出丢帧

```bash
lvi_eval run --run results/hk01 --gt gt/HKisland01.tum                 # 单次运行 → report.md + 图
scripts/eval_batch.sh -c config/livox_avia.yaml -d ~/datasets/mars_lvig \
    -g ~/datasets/mars_lvig/gt -n 3                                      # 全数据集 × 3 次重复
lvi_eval compare results/<旧> results/<新> --fail-on-regression          # 改动前后对比
```

## 运行

```bash
roslaunch lvi_slam lvi_slam.launch          # ROS1
ros2 launch lvi_slam lvi_slam.launch.py     # ROS2
```

**第一次跑务必按这个顺序**：

1. `config/livox_mid360.yaml` 里 `visual.enable: false`，先跑纯 LIO
2. 开机**保持静止约 1 秒**（静止初始化要估重力方向和陀螺 bias）
3. 用 `lvi_eval` 确认 LIO 精度到位了，再开视觉

数学框架没立住之前加模块，debug 会变成噩梦。

## 实现状态

| 模块 | 状态 |
|---|---|
| SO(3) 流形运算 / 状态定义 | ✅ |
| Livox 预处理（tag 滤波、时间偏移） | ✅ |
| 数据同步（LIO/VIO 交替、点云切分） | ✅ |
| IMU 传播 + 协方差 + 去畸变 | ✅ |
| 统一体素地图（自适应八叉树 + plane_var） | ✅ |
| iEKF（对偶增益 + 退化分析） | ✅ |
| LIO 更新（点-面残差 + 自适应权重） | ✅ |
| 直接法 VIO（光度残差 + 平面单应 warp + 参考帧优选） | ✅ |
| 特征点法备轨 + 健康度切换 | ✅ |
| 后端因子图（ISAM2 + 里程计/GPS/回环因子） | ✅ |
| 回环检测（Scan Context + ICP + 视觉双模态） | ✅ |
| ROS2 节点 + launch + 配置 | ✅ |
| ROS1 节点 + launch + bag 读取 | ✅（节点与 launch 待实机验证） |
| 离线运行器（ROS1/ROS2 bag、KITTI）+ 诊断输出 | ✅ |
| 评估工具 `lvi_eval` | ✅ |
| Docker（noetic/humble/noros × amd64/arm64）+ CI | ✅（待首次在本机/CI 上实际构建） |
| 外参在线标定解锁逻辑 | 🚧 接口已留 |
| RViz 配置文件 | 🚧 |

## 纪律

**不要手工调残差权重。** 一开始麻烦，换传感器时会感谢自己。

**不要过早并行前端。** 先跑对，profile 出真实瓶颈再说。

**评估基础设施优先于任何新特性。** 每次改动都用 `scripts/eval_batch.sh -b <上次结果>` 跑一遍，否则你不知道是变好还是变坏。

**时间同步从第一天就做对。** `td` 在线估计不是可选项。

## 后续可加的学习模块（按投入产出排序）

框架已为这些留好了位置，都是加法不是替换：

1. **学习不确定度** —— `R_inv` 上乘一个网络预测的可靠性权重 `w∈(0,1]`，覆盖噪声模型管不到的模型失配（树叶、玻璃、动态物体）。接口零侵入，可随时关掉退化回纯几何。
2. **动态物体剔除** —— 轻量语义分割 mask 投影回点云，`buildResidualList` 里加一行 `if (pv.is_dynamic) continue;`。完全不触碰数学框架。
3. **学习回环描述子** —— Scan Context 换成 BEVPlace++ / LoGG3D-Net，接口不变。检索用学习，几何验证仍用 ICP。
4. **GFM 兜底** —— 直接法和特征法都失效时（进隧道、强逆光），用 MASt3R/VGGT 做一次两帧配准给粗略相对位姿。噪声设很大，只防发散不做精度来源。

## License

MIT
