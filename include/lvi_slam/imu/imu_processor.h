// lvi_slam/imu/imu_processor.h
// IMU 前向传播 + 反向去畸变。
//
// 与 VINS-Mono 的预积分不同：这里不做预积分。
// 预积分是为优化框架服务的（避免优化迭代中重复积分），滤波框架下状态是
// 单点估计，直接前向传播协方差即可，不需要构造与初始状态无关的增量。
#pragma once

#include "lvi_slam/common/types.h"
#include "lvi_slam/common/config.h"

namespace lvi {

class ImuProcessor {
 public:
  explicit ImuProcessor(const Config& cfg);

  // 静止初始化：估计初始重力方向和陀螺 bias。
  // 返回 true 表示初始化完成。
  bool tryInitialize(const std::vector<ImuData>& imus, State* state);

  bool initialized() const { return initialized_; }

  // 前向传播 + 点云去畸变。
  //
  // 传播：从 state->timestamp 积分到 pkg.sync_time，同时传播协方差。
  // 去畸变：反向遍历 IMU 缓存，把每个点变换到 sync_time 时刻的 body 系。
  //
  // 去畸变基准选 sync_time（帧末对齐图像时刻），而非 LIO-SAM 的帧首。
  // 这样 LIO 更新和 VIO 更新的状态时刻严格一致。
  void process(const SyncPackage& pkg, State* state, CloudPtr cloud_undistort);

  // 高频位姿前推（供外部控制器使用，不影响滤波状态）
  void propagateOnce(const ImuData& imu, double dt, State* s) const;

  // 重力对齐：把估计的重力方向旋到 (0,0,-1)，让世界系 Z 轴朝上
  void alignGravity(State* state);

 private:
  // 单步中值积分 + 协方差传播
  void propagateWithCov(const ImuData& prev, const ImuData& curr,
                        double dt, State* s) const;

  // 反向去畸变：用缓存的 IMU 位姿把每个点变换到基准时刻
  void undistortCloud(const SyncPackage& pkg, const State& end_state,
                      CloudPtr cloud) const;

  Config cfg_;
  bool   initialized_ = false;
  bool   gravity_aligned_ = false;

  V3D    mean_acc_ = V3D::Zero();
  V3D    mean_gyr_ = V3D::Zero();
  int    init_count_ = 0;

  ImuData last_imu_;

  // 去畸变用：本轮每个 IMU 时刻的位姿快照
  struct ImuPose {
    double t;
    V3D    acc, gyr, vel, pos;
    M3D    rot;
  };
  mutable std::vector<ImuPose> imu_poses_;

  // 过程噪声矩阵 Q (12x12)：陀螺噪声、加速度噪声、两个 bias 随机游走
  Eigen::Matrix<double, 12, 12> Q_;
};

}  // namespace lvi
