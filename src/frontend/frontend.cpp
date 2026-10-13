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

  // 初始协方差。
  // 地图由首帧点云建立，世界系就锚定在首帧位姿上，所以首帧位姿「相对地图」
  // 是精确已知的 —— xyz 和 yaw 虽然全局不可观，但在本滤波器里不需要大方差。
  // （LIO-SAM 先验因子给 1e8 是因为它的地图不锚定在首帧，两者不能类比。）
  // 而且位姿方差会传播进每个点的观测噪声（LioUpdater::propagatePointCov），
  // 这里给大方差会让 LIO 观测权重趋近于 0，系统退化成纯 IMU 积分。
  // 量级参考 FAST-LIO / VoxelMap。
  state_.cov.setIdentity();
  state_.cov.block<3, 3>(kIdxRot, kIdxRot)     *= 1e-4;
  state_.cov.block<3, 3>(kIdxPos, kIdxPos)     *= 1e-4;
  state_.cov.block<3, 3>(kIdxBg, kIdxBg)       *= 1e-4;   // 静止初始化已估计
  state_.cov.block<3, 3>(kIdxBa, kIdxBa)       *= 1e-3;
  state_.cov.block<3, 3>(kIdxGrav, kIdxGrav)   *= 1e-5;   // 静止初始化已估计
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
  diag_ = FrameDiag();

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
  if (!ok) { fillDiag(pkg, false); return false; }

  // ---- 4. 退化分析 ----
  degen_ = iekf_->analyzeDegeneracy(state_);
  fillDiag(pkg, true);   // 记录的是跨模态补偿之前、更新刚结束时的协方差
  // 跨模态补偿只在有另一个模态可补时才有意义。纯 LIO 下放大协方差只会
  // 让下一帧继续判定为退化、继续放大（正反馈），最终把 LIO 权重压到 0。
  if (degen_.is_degenerate && vio_) adaptCrossModalWeights();

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

  kf_poses_.push_back({kf->id, state_.rot, state_.pos});
  while (kf_poses_.size() > kKfPoseHistory) kf_poses_.pop_front();

  if (kf_callback_) kf_callback_(kf);
}

bool Frontend::keyframePose(int id, M3D* rot, V3D* pos) const {
  // id 连续递增，直接按偏移定位
  if (kf_poses_.empty()) return false;
  const int idx = id - kf_poses_.front().id;
  if (idx < 0 || idx >= static_cast<int>(kf_poses_.size())) return false;
  *rot = kf_poses_[idx].rot;
  *pos = kf_poses_[idx].pos;
  return true;
}

void Frontend::fillDiag(const SyncPackage& pkg, bool ok) {
  diag_.t    = state_.timestamp;
  diag_.mode = pkg.mode == UpdateMode::kLio ? 1 : 2;
  diag_.ok   = ok;
  const auto& st = iekf_->lastStats();
  diag_.iterations = st.iterations;
  diag_.num_valid  = st.num_valid;
  diag_.mean_error = st.mean_error;
  diag_.converged  = st.converged;
  diag_.degenerate_mask = 0;
  if (ok) {
    for (int d : degen_.degenerate_dims) diag_.degenerate_mask |= 1 << d;
  }
  if (vio_ && pkg.mode == UpdateMode::kVio) {
    diag_.vio_track =
        vio_->currentTrack() == HybridVisualUpdater::Track::kDirect ? 0 : 1;
  }
  diag_.pose_cov = state_.cov.block<6, 6>(0, 0);
  diag_.td = state_.td;
  diag_.bg = state_.bg;
  diag_.ba = state_.ba;
}

void Frontend::applyLoopCorrection(const std::vector<KeyframePtr>& corrected_kfs,
                                   int anchor_kf_id,
                                   const M3D& anchor_rot, const V3D& anchor_pos) {
  M3D kf_rot;
  V3D kf_pos;
  if (!keyframePose(anchor_kf_id, &kf_rot, &kf_pos)) {
    std::cerr << "[Frontend] loop correction dropped: keyframe " << anchor_kf_id
              << " not in pose history\n";
    return;
  }

  // 修正变换 T_c = T_opt · T_odom⁻¹：把前端世界系映射到回环修正后的世界系。
  // 作用到状态上要连同速度、重力一起旋转，否则 IMU 传播在新坐标系下不自洽。
  const M3D R_c = anchor_rot * kf_rot.transpose();
  const V3D t_c = anchor_pos - R_c * kf_pos;

  // 1. 用后端修正过的关键帧位姿重建前端局部地图。
  //    只重建最近 N 帧覆盖的范围，代价可控。
  map_->rebuildFromKeyframes(corrected_kfs);

  // 2. 把 iEKF 状态搬到修正后的坐标系。
  //    协方差不重置——回环降低了不确定度，但这个信息量化地传回滤波器
  //    需要更严谨的处理（当前版本先保守地保持原协方差）。
  state_.rot  = R_c * state_.rot;
  state_.pos  = R_c * state_.pos + t_c;
  state_.vel  = R_c * state_.vel;
  state_.grav = R_c * state_.grav;

  last_kf_rot_ = R_c * last_kf_rot_;
  last_kf_pos_ = R_c * last_kf_pos_ + t_c;
  last_loop_kf_pos_ = R_c * last_loop_kf_pos_ + t_c;
  for (auto& kp : kf_poses_) {
    kp.rot = R_c * kp.rot;
    kp.pos = R_c * kp.pos + t_c;
  }

  std::cout << "[Frontend] loop correction applied (|dt|="
            << t_c.norm() << " m, |dR|=" << so3::Log(R_c).norm() * 57.29578
            << " deg), local map rebuilt from " << corrected_kfs.size()
            << " keyframes\n";
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
