#include "lvi_slam/lio/lio_updater.h"
#include <omp.h>

namespace lvi {

LioUpdater::LioUpdater(const Config& cfg, VoxelMap* map)
    : cfg_(cfg), map_(map) {}

void LioUpdater::setInputCloud(const CloudPtr& cloud_body) {
  cloud_body_ = cloud_body;
  const size_t n = cloud_body_ ? cloud_body_->size() : 0;

  pv_list_.assign(n, PointWithVar());
  body_cov_list_.assign(n, M3D::Zero());
  cross_mat_list_.assign(n, M3D::Zero());

  // 预计算每个点的 body 系协方差和叉乘矩阵（不依赖状态，只算一次）
  const M3D& Rli = cfg_.lidar.ext_rot_li;
  const V3D& tli = cfg_.lidar.ext_pos_li;

  for (size_t i = 0; i < n; ++i) {
    V3D p(cloud_body_->points[i].x,
          cloud_body_->points[i].y,
          cloud_body_->points[i].z);
    if (std::abs(p.z()) < 1e-6) p.z() = 1e-3;

    M3D cov;
    VoxelMap::calcPointCov(p, cfg_.lidar.depth_err, cfg_.lidar.beam_err, &cov);
    body_cov_list_[i] = cov;

    const V3D p_imu = Rli * p + tli;          // LiDAR 系 -> IMU 系
    cross_mat_list_[i] = so3::Hat(p_imu);
    pv_list_[i].point_b  = p_imu;
    pv_list_[i].body_var = cov;
  }
}

void LioUpdater::propagatePointCov(const State& state) {
  const M3D rot_var = state.cov.block<3, 3>(kIdxRot, kIdxRot);
  const M3D pos_var = state.cov.block<3, 3>(kIdxPos, kIdxPos);
  const M3D& R = state.rot;
  const M3D& Rli = cfg_.lidar.ext_rot_li;

  const size_t n = pv_list_.size();
  for (size_t i = 0; i < n; ++i) {
    pv_list_[i].point_w = R * pv_list_[i].point_b + state.pos;

    const M3D& px = cross_mat_list_[i];
    const M3D RR = R * Rli;
    // 世界系协方差 = 测量噪声传播 + 姿态不确定度 + 位置不确定度
    pv_list_[i].var = RR * body_cov_list_[i] * RR.transpose()
                    + px * rot_var * px.transpose()
                    + pos_var;
  }
}

bool LioUpdater::buildResidual(const State& state, Residuals* out) {
  if (!cloud_body_ || cloud_body_->empty() || map_ == nullptr) return false;

  // 1. 用当前状态把点变换到世界系，并传播协方差
  propagatePointCov(state);

  // 2. 在地图中找对应平面（OpenMP 并行，内部用预分配数组避免竞争）
  ptpl_list_.clear();
  map_->buildResidualList(pv_list_, &ptpl_list_);

  const int m = static_cast<int>(ptpl_list_.size());
  last_valid_count_ = m;
  if (m < 20) return false;

  // 3. LiDAR 只观测位姿，H 的有效列就是 rot(3) + pos(3)
  out->active_cols = {kIdxRot + 0, kIdxRot + 1, kIdxRot + 2,
                      kIdxPos + 0, kIdxPos + 1, kIdxPos + 2};
  out->H_sub.setZero(m, 6);
  out->z.setZero(m);
  out->R_inv.setZero(m);
  out->num_valid = m;

  double total_err = 0.0;
  const M3D& Rli = cfg_.lidar.ext_rot_li;

  #pragma omp parallel for num_threads(cfg_.num_threads) reduction(+:total_err)
  for (int i = 0; i < m; ++i) {
    const auto& ptpl = ptpl_list_[i];
    const V3D p_imu = Rli * ptpl.point_b + cfg_.lidar.ext_pos_li;
    const M3D px = so3::Hat(p_imu);

    // ---- 观测噪声：两项之和 ----
    // (a) 平面参数不确定度沿残差方向的投影
    Eigen::Matrix<double, 1, 6> J_nq;
    J_nq.block<1, 3>(0, 0) = ptpl.point_w - ptpl.center;   // 对法向量的导数
    J_nq.block<1, 3>(0, 3) = -ptpl.normal;                 // 对中心点的导数
    const double sigma_plane = (J_nq * ptpl.plane_var * J_nq.transpose())(0, 0);

    // (b) 点测量不确定度沿法向量方向的投影
    const M3D RR = state.rot * Rli;
    const M3D pt_var = RR * ptpl.body_cov * RR.transpose();
    const double sigma_point = ptpl.normal.transpose() * pt_var * ptpl.normal;

    out->R_inv(i) = 1.0 / (1e-3 + sigma_plane + sigma_point);

    // ---- 雅可比：d(n^T (R p + t) + d) / d[dphi, dt] ----
    // 对旋转：n^T * d(R p)/d(dphi) = n^T * (-R [p]_x) = ([p]_x R^T n)^T
    const V3D J_rot = px * state.rot.transpose() * ptpl.normal;
    out->H_sub.row(i) << J_rot.x(), J_rot.y(), J_rot.z(),
                         ptpl.normal.x(), ptpl.normal.y(), ptpl.normal.z();
    out->z(i) = -ptpl.dist;
    total_err += std::abs(ptpl.dist);
  }

  out->mean_error = total_err / m;
  return true;
}

void LioUpdater::updateMap(const State& state) {
  // 用收敛后的状态重新算世界系位置和协方差，再灌进地图
  propagatePointCov(state);
  map_->update(pv_list_);

  if (cfg_.voxel.sliding_en) {
    map_->slideMap(state.pos);
  }
}

}  // namespace lvi
