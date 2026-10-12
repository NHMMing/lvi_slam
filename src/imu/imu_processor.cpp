// lvi_slam/imu/imu_processor.cpp
// IMU 前向传播 + 反向去畸变。
//
// 这里不做预积分。预积分是为优化框架服务的（避免迭代中重复积分），
// 滤波框架下状态是单点估计，直接前向传播协方差即可，
// 不需要构造与初始状态无关的增量。少写几百行还更快。
#include "lvi_slam/imu/imu_processor.h"
#include <algorithm>

namespace lvi {

ImuProcessor::ImuProcessor(const Config& cfg) : cfg_(cfg) {
  Q_.setZero();
  Q_.block<3, 3>(0, 0) = M3D::Identity() * cfg.imu.gyr_noise * cfg.imu.gyr_noise;
  Q_.block<3, 3>(3, 3) = M3D::Identity() * cfg.imu.acc_noise * cfg.imu.acc_noise;
  Q_.block<3, 3>(6, 6) = M3D::Identity() * cfg.imu.gyr_bias_noise * cfg.imu.gyr_bias_noise;
  Q_.block<3, 3>(9, 9) = M3D::Identity() * cfg.imu.acc_bias_noise * cfg.imu.acc_bias_noise;
  imu_poses_.reserve(512);
}

// 静止初始化。要求开机时设备静止约 1 秒。
//
// 不做 VINS-Mono 那套 SFM + 视觉惯性对齐 —— LiDAR 直接给了尺度，
// 唯一需要初始化的是重力方向和陀螺 bias，静置几秒就够。
// 代价是要求开机静止；收益是初始化代码从几百行降到几十行，且不会初始化失败。
bool ImuProcessor::tryInitialize(const std::vector<ImuData>& imus, State* state) {
  for (const auto& imu : imus) {
    ++init_count_;
    // 增量式均值，不用存全部数据
    mean_acc_ += (imu.acc - mean_acc_) / static_cast<double>(init_count_);
    mean_gyr_ += (imu.gyr - mean_gyr_) / static_cast<double>(init_count_);
    last_imu_ = imu;
  }
  if (init_count_ < cfg_.imu.init_frame_num) return false;

  const double acc_norm = mean_acc_.norm();
  if (acc_norm < 1e-3) return false;

  // 加速度单位自适应：Livox 内置 IMU（Avia/Horizon）输出单位是 g 而非 m/s^2。
  // 静止时 |acc| 应等于当地重力，据此求比例系数，之后所有加速度读数都乘上它。
  // 已经是 m/s^2 的 IMU 这里约等于 1，相当于顺带做了一次模长校正。
  acc_scale_ = cfg_.imu.gravity_norm / acc_norm;

  // 静止时加速度计读数就是 -g（IMU 系下）
  state->grav = -mean_acc_ / acc_norm * cfg_.imu.gravity_norm;
  state->bg = mean_gyr_;     // 静止时陀螺读数即 bias
  state->ba.setZero();       // 加速度 bias 与重力在静止时不可分，留给后续收敛
  state->vel.setZero();
  state->timestamp = last_imu_.t;

  initialized_ = true;
  return true;
}

// 重力对齐：把世界系 Z 轴转到与重力反向。
// 纯粹为了输出位姿符合直觉，不影响估计精度。
void ImuProcessor::alignGravity(State* state) {
  if (gravity_aligned_) return;
  const V3D target(0, 0, -1);
  const M3D R = Eigen::Quaterniond::FromTwoVectors(
      state->grav.normalized(), target).toRotationMatrix();

  state->rot = R * state->rot;
  state->pos = R * state->pos;
  state->vel = R * state->vel;
  state->grav = R * state->grav;
  gravity_aligned_ = true;
}

// 单步中值积分 + 协方差传播。
// 中值法（前后两个读数取平均）是二阶精度，在 200Hz IMU 上比欧拉法明显更准，
// 计算量几乎不增加。
void ImuProcessor::propagateWithCov(const ImuData& prev, const ImuData& curr,
                                    double dt, State* s) const {
  if (dt <= 0.0 || dt > 0.5) return;   // 异常 dt 直接跳过，防止发散

  const V3D gyr_mid = 0.5 * (prev.gyr + curr.gyr) - s->bg;
  const V3D acc_raw_mid = 0.5 * (prev.acc + curr.acc) * acc_scale_ - s->ba;

  const M3D R_old = s->rot;
  const M3D dR = so3::Exp(gyr_mid, dt);
  const M3D R_new = R_old * dR;

  // 加速度也取中值：用新旧两个旋转分别转到世界系后平均
  const V3D acc_w = 0.5 * (R_old * acc_raw_mid + R_new * acc_raw_mid) + s->grav;

  // ---- 误差状态协方差传播 ----
  // 先用旧状态构造 F、G，再更新名义状态（顺序不能反）
  StateMat F = StateMat::Identity();
  F.block<3, 3>(kIdxRot, kIdxRot)  = dR.transpose();
  F.block<3, 3>(kIdxRot, kIdxBg)   = -so3::Jr(gyr_mid * dt) * dt;
  F.block<3, 3>(kIdxPos, kIdxVel)  = M3D::Identity() * dt;
  F.block<3, 3>(kIdxVel, kIdxRot)  = -R_old * so3::Hat(acc_raw_mid) * dt;
  F.block<3, 3>(kIdxVel, kIdxBa)   = -R_old * dt;
  F.block<3, 3>(kIdxVel, kIdxGrav) = M3D::Identity() * dt;

  Eigen::Matrix<double, kStateDim, 12> G = Eigen::Matrix<double, kStateDim, 12>::Zero();
  G.block<3, 3>(kIdxRot, 0) = -so3::Jr(gyr_mid * dt) * dt;
  G.block<3, 3>(kIdxVel, 3) = -R_old * dt;
  G.block<3, 3>(kIdxBg, 6)  = M3D::Identity() * dt;
  G.block<3, 3>(kIdxBa, 9)  = M3D::Identity() * dt;

  // Q_ 存的是连续时间噪声密度的平方（config 里的单位是 /sqrt(Hz)）。
  // 离散化后单步方差应为 σ²·dt，而 G 里已经带了一个 dt（G Q Gᵀ ∝ dt²），
  // 所以这里除以 dt。漏掉这一步过程噪声会小 1/dt 倍（200Hz 下 200 倍），
  // 滤波器过度自信，卡方检验会把正确的点面匹配也剔掉。
  s->cov = F * s->cov * F.transpose() + G * (Q_ / dt) * G.transpose();

  // ---- 名义状态传播 ----
  s->pos += s->vel * dt + 0.5 * acc_w * dt * dt;
  s->vel += acc_w * dt;
  s->rot = R_new;
  s->timestamp = curr.t;
}

void ImuProcessor::propagateOnce(const ImuData& imu, double dt, State* s) const {
  if (dt <= 0.0 || dt > 0.5) return;
  const V3D gyr = imu.gyr - s->bg;
  const V3D acc = s->rot * (imu.acc * acc_scale_ - s->ba) + s->grav;
  s->rot = s->rot * so3::Exp(gyr, dt);
  s->pos += s->vel * dt + 0.5 * acc * dt * dt;
  s->vel += acc * dt;
  s->timestamp = imu.t;
}

void ImuProcessor::process(const SyncPackage& pkg, State* state,
                           CloudPtr cloud_undistort) {
  if (pkg.imus.empty()) return;

  imu_poses_.clear();
  imu_poses_.push_back({state->timestamp, last_imu_.acc, last_imu_.gyr,
                        state->vel, state->pos, state->rot});

  ImuData prev = last_imu_;
  if (prev.t < 0) prev = pkg.imus.front();

  for (const auto& curr : pkg.imus) {
    propagateWithCov(prev, curr, curr.t - prev.t, state);
    imu_poses_.push_back({curr.t, curr.acc, curr.gyr,
                          state->vel, state->pos, state->rot});
    prev = curr;
  }

  // 传播到同步时刻。最后一个 IMU 可能落在 sync_time 之前，补这一小段。
  // 不补的话去畸变的基准时刻和状态时刻对不上，会有半个 IMU 周期的系统误差。
  if (pkg.sync_time > prev.t) {
    propagateWithCov(prev, prev, pkg.sync_time - prev.t, state);
    imu_poses_.push_back({pkg.sync_time, prev.acc, prev.gyr,
                          state->vel, state->pos, state->rot});
  }
  last_imu_ = prev;
  state->timestamp = pkg.sync_time;

  if (pkg.cloud && !pkg.cloud->empty()) {
    *cloud_undistort = *pkg.cloud;
    undistortCloud(pkg, *state, cloud_undistort);
  }
}

// 反向去畸变。
//
// 基准选帧末（= sync_time = 图像曝光时刻），不是 LIO-SAM 的帧首。
// 这样 LIO 更新和 VIO 更新作用在同一时刻的状态上，两者严格一致。
//
// 与 LIO-SAM 不同，这里同时补偿旋转和平移。LIO-SAM 把平移补偿注释掉了，
// 理由是"步行速度下收益很小"—— 但车载 60km/h 时 100ms 移动 1.7m，
// 平移畸变不能忽略。补偿的代价只是几次向量运算。
void ImuProcessor::undistortCloud(const SyncPackage& pkg, const State& end_state,
                                  CloudPtr cloud) const {
  if (!cloud || cloud->empty() || imu_poses_.size() < 2) return;

  const M3D R_end_inv = end_state.rot.transpose();
  const V3D p_end = end_state.pos;
  const M3D& R_il = cfg_.lidar.ext_rot_li;
  const V3D& p_il = cfg_.lidar.ext_pos_li;

  // 按时间偏移升序排列，便于单调扫描 IMU 位姿
  std::sort(cloud->points.begin(), cloud->points.end(),
            [](const PointType& a, const PointType& b) {
              return a.curvature < b.curvature;
            });

  const double base_time = pkg.sync_time - 
      (cloud->points.back().curvature - cloud->points.front().curvature) * 1e-3;

  size_t idx = 0;
  for (auto& p : cloud->points) {
    const double pt_time = base_time + (p.curvature - cloud->points.front().curvature) * 1e-3;

    while (idx + 2 < imu_poses_.size() && imu_poses_[idx + 1].t < pt_time) ++idx;

    // 在两个 IMU 位姿之间插值出该点采集时刻的位姿
    const auto& a = imu_poses_[idx];
    const auto& b = imu_poses_[std::min(idx + 1, imu_poses_.size() - 1)];
    const double span = b.t - a.t;
    const double r = (span > 1e-9) ? std::clamp((pt_time - a.t) / span, 0.0, 1.0) : 0.0;

    const M3D R_i = a.rot * so3::Exp(so3::Log(a.rot.transpose() * b.rot) * r);
    const V3D p_i = a.pos + (b.pos - a.pos) * r;

    // LiDAR 系 → IMU 系 → 世界系（采集时刻）→ IMU 系（帧末）
    const V3D p_lidar(p.x, p.y, p.z);
    const V3D p_imu = R_il * p_lidar + p_il;
    const V3D p_world = R_i * p_imu + p_i;
    const V3D p_body = R_end_inv * (p_world - p_end);

    p.x = static_cast<float>(p_body.x());
    p.y = static_cast<float>(p_body.y());
    p.z = static_cast<float>(p_body.z());
  }
}

}  // namespace lvi
