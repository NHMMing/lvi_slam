#include "lvi_slam/frontend/frontend.h"
#include <pcl/filters/voxel_grid.h>
#include <iostream>

namespace lvi {

namespace {
// 相机内参在 YAML 里挂在 visual 下，CameraModel 只需要其中的内参部分
CameraConfig toCameraConfig(const VisualConfig& v) {
  CameraConfig c;
  c.width = v.width;
  c.height = v.height;
  c.fx = v.fx;
  c.fy = v.fy;
  c.cx = v.cx;
  c.cy = v.cy;
  c.d0 = v.k1;
  c.d1 = v.k2;
  c.d2 = v.p1;
  c.d3 = v.p2;
  return c;
}
}  // namespace

Frontend::Frontend(const Config& cfg) : cfg_(cfg) {
  imu_  = std::make_unique<ImuProcessor>(cfg);
  map_  = std::make_unique<VoxelMap>(cfg);
  cam_  = std::make_unique<CameraModel>(toCameraConfig(cfg.visual));

  IEKF::Options opt;
  opt.max_iterations = cfg.max_iterations;
  iekf_ = std::make_unique<IEKF>(opt);

  lio_  = std::make_unique<LioUpdater>(cfg, map_.get());
  if (cfg.visual.enable) {
    vio_ = std::make_unique<HybridVisualUpdater>(cfg, map_.get(), cam_.get());
  }

  cloud_undistort_.reset(new PointCloud());
  cloud_down_.reset(new PointCloud());

  state_.ext_rot_ci = cfg.extrinsic.ext_rot_ci;
  state_.ext_pos_ci = cfg.extrinsic.ext_pos_ci;

  // 初始协方差：编码可观性分析的结论。
  // 4 个不可观自由度（xyz + yaw）给大方差，roll/pitch 给小方差
  // （IMU 加速度计感知重力方向，roll/pitch 绝对可观）。
  // 这和 LIO-SAM 第一帧先验因子的噪声 [1e-2,1e-2,pi^2,1e8,1e8,1e8] 是同一个道理。
  state_.cov.setIdentity();
  state_.cov.block<2, 2>(kIdxRot, kIdxRot)     *= 1e-4;   // roll, pitch
  state_.cov(kIdxRot + 2, kIdxRot + 2)          = 1e4;    // yaw 不可观
  state_.cov.block<3, 3>(kIdxPos, kIdxPos)     *= 1e4;    // 全局位置不可观
  state_.cov.block<3, 3>(kIdxExtRot, kIdxExtRot) *= 1e-6; // 外参：极小过程噪声
  state_.cov.block<3, 3>(kIdxExtPos, kIdxExtPos) *= 1e-6;
}

Frontend::~Frontend() = default;

// IMU 对齐完成且首帧点云已建图之后，状态才有意义，才对外发布位姿
bool Frontend::initialized() const {
  return imu_->initialized() && map_initialized_;
}

bool Frontend::process(const SyncPackage& pkg) {
  if (system_start_time_ < 0) system_start_time_ = pkg.sync_time;

  // ---- 1. IMU 初始化（静止估重力和陀螺 bias）----
  if (!imu_->initialized()) {
    if (!imu_->tryInitialize(pkg.imus, &state_)) return false;
    imu_->alignGravity(&state_);
    return false;
  }

  // ---- 2. IMU 前向传播 + 点云去畸变 ----
  cloud_undistort_->clear();
  imu_->process(pkg, &state_, cloud_undistort_);
  prior_ = state_;                    // 先验固定，iEKF 迭代中不变

  updateExtrinsicUnlock();

  // ---- 3. 按模式做量测更新 ----
  bool ok = false;
  switch (pkg.mode) {
    case UpdateMode::kLio: ok = handleLio(pkg); break;
    case UpdateMode::kVio: ok = handleVio(pkg); break;
    default: return false;
  }
  if (!ok) return false;

  // ---- 4. 退化分析 ----
  degen_ = iekf_->analyzeDegeneracy(state_);
  if (degen_.is_degenerate) adaptCrossModalWeights();

  // ---- 5. 关键帧产出 ----
  if (shouldCreateKeyframe()) createKeyframe(pkg);

  return true;
}

bool Frontend::handleLio(const SyncPackage& pkg) {
  if (cloud_undistort_->empty()) return false;

  // 体素降采样。注意：只降采样用于配准的点，
  // 关键帧存的是稠密点（回环 ICP 和最终建图需要）。
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(cfg_.lidar.filter_size, cfg_.lidar.filter_size,
                 cfg_.lidar.filter_size);
  vg.setInputCloud(cloud_undistort_);
  cloud_down_->clear();
  vg.filter(*cloud_down_);

  lio_->setInputCloud(cloud_down_);

  // 首帧：地图还是空的，直接用当前点云建图，不做更新
  if (!map_initialized_) {
    lio_->buildResidual(state_, nullptr);   // 只为填充 pv_list
    lio_->updateMap(state_);
    map_initialized_ = true;
    return true;
  }

  if (!iekf_->update(lio_.get(), prior_, &state_)) {
    std::cerr << "[Frontend] LIO update failed\n";
    return false;
  }
  lio_->updateMap(state_);
  return true;
}

bool Frontend::handleVio(const SyncPackage& pkg) {
  if (!vio_ || pkg.image.gray.empty()) return false;

  // 视觉更新紧跟在 LIO 之后。这个顺序不能反——
  // 直接法的光度误差收敛域只有几个像素，必须靠 LiDAR 提供准确初值。
  ObservationModel* model = vio_->prepare(
      pkg.image.gray, pkg.sync_time, state_, lio_->pointsWithVar());
  if (model == nullptr) return false;

  if (!iekf_->update(model, prior_, &state_)) {
    std::cerr << "[Frontend] VIO update failed, track="
              << (vio_->currentTrack() == HybridVisualUpdater::Track::kDirect
                      ? "direct" : "feature") << "\n";
    return false;
  }
  vio_->finalize(state_);
  return true;
}

bool Frontend::shouldCreateKeyframe() const {
  if (last_kf_time_ < 0) return true;   // 第一帧必是关键帧

  const M3D dR = last_kf_rot_.transpose() * state_.rot;
  const double angle = so3::Log(dR).norm();
  const double dist  = (state_.pos - last_kf_pos_).norm();
  const double dt    = state_.timestamp - last_kf_time_;

  // 质量门槛：协方差过大的帧不做关键帧，避免把坏位姿灌进因子图
  const double cov_trace = state_.cov.block<6, 6>(0, 0).trace();
  if (cov_trace > cfg_.keyframe.cov_thresh && dt < cfg_.keyframe.time_thresh) {
    return false;
  }

  return dist  > cfg_.keyframe.dist_thresh
      || angle > cfg_.keyframe.angle_thresh
      || dt    > cfg_.keyframe.time_thresh;   // 静止兜底
}

void Frontend::createKeyframe(const SyncPackage& pkg) {
  auto kf = std::make_shared<Keyframe>();
  kf->id  = kf_id_counter_++;
  kf->t   = state_.timestamp;
  kf->rot = state_.rot;
  kf->pos = state_.pos;
  kf->pose_cov = state_.cov.block<6, 6>(0, 0);

  // 存稠密点云（不是降采样后的），回环 ICP 和最终建图需要
  kf->cloud_body.reset(new PointCloud(*cloud_undistort_));

  if (vio_) kf->descriptors = vio_->loopDescriptors();

  // 回环检索库比建图关键帧稀疏。
  // 太密会让 Scan Context 数据库膨胀、检索变慢；
  // 太稀会漏检回环。经验值是每 3-5 米一个。
  const double d_loop = (state_.pos - last_loop_kf_pos_).norm();
  if (d_loop > cfg_.keyframe.loop_kf_stride || kf->id == 0) {
    kf->is_loop_candidate = true;
    last_loop_kf_pos_ = state_.pos;
  }

  last_kf_rot_  = state_.rot;
  last_kf_pos_  = state_.pos;
  last_kf_time_ = state_.timestamp;

  if (kf_callback_) kf_callback_(kf);
}

void Frontend::applyLoopCorrection(const std::vector<KeyframePtr>& corrected_kfs,
                                   const M3D& corrected_rot,
                                   const V3D& corrected_pos) {
  // 1. 用后端修正过的关键帧位姿重建前端局部地图。
  //    只重建最近 N 帧覆盖的范围，代价可控。
  map_->rebuildFromKeyframes(corrected_kfs);

  // 2. 把 iEKF 状态搬到修正后的位姿。
  //    协方差不重置——回环降低了不确定度，但这个信息量化地传回滤波器
  //    需要更严谨的处理（当前版本先保守地保持原协方差）。
  state_.rot = corrected_rot;
  state_.pos = corrected_pos;

  last_kf_rot_ = corrected_rot;
  last_kf_pos_ = corrected_pos;

  std::cout << "[Frontend] loop correction applied, local map rebuilt from "
            << corrected_kfs.size() << " keyframes\n";
}

void Frontend::adaptCrossModalWeights() {
  // LiDAR 退化的方向，让视觉补；视觉退化的方向，让 LiDAR 补。
  //
  // 具体做法：把退化维度对应的过程噪声调大，等价于降低该方向上
  // 先验的置信度，让另一个模态的观测在该方向获得更大的相对权重。
  //
  // 典型场景：
  //   长走廊 -> LiDAR 沿走廊方向无约束，但墙上纹理给视觉充分约束
  //   白墙   -> 视觉无纹理，但 LiDAR 有平面约束
  for (int dim : degen_.degenerate_dims) {
    state_.cov(dim, dim) *= 4.0;
  }
}

void Frontend::updateExtrinsicUnlock() {
  if (cfg_.extrinsic.online_calib_level == 0) return;
  const double elapsed = state_.timestamp - system_start_time_;

  // 分阶段解锁：先旋转后平移。
  // 平移外参的可观性远弱于旋转，过早解锁会被噪声带偏。
  if (!ext_rot_unlocked_ && elapsed > cfg_.extrinsic.unlock_rot_after_sec) {
    ext_rot_unlocked_ = true;
    std::cout << "[Frontend] extrinsic rotation unlocked for online calib\n";
  }
  if (cfg_.extrinsic.online_calib_level >= 2 && !ext_pos_unlocked_ &&
      elapsed > cfg_.extrinsic.unlock_pos_after_sec) {
    ext_pos_unlocked_ = true;
    std::cout << "[Frontend] extrinsic translation unlocked\n";
  }
}

State Frontend::propagateHighFreq(const ImuData& imu) {
  static State hf_state;
  static double last_t = -1;
  if (last_t < 0) { hf_state = state_; last_t = imu.t; return hf_state; }
  imu_->propagateOnce(imu, imu.t - last_t, &hf_state);
  last_t = imu.t;
  return hf_state;
}

}  // namespace lvi
