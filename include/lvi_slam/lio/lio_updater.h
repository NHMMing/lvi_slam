// lvi_slam/lio/lio_updater.h
// LiDAR 观测模型：点到平面残差。
//
// 与 LIO-SAM 的关键区别在观测噪声 R 的来源。
// LIO-SAM:      s = 1 - 0.9 * |dist|         <- 手工设计的线性衰减，靠调参
// 本实现:       R = sigma_plane + n^T Σ_p n  <- 从传感器噪声模型严格推导
//
// 后者的好处是换传感器只需改配置文件里的 depth_err / beam_err，
// 不需要重新调权重曲线。
#pragma once

#include "lvi_slam/common/types.h"
#include "lvi_slam/common/config.h"
#include "lvi_slam/estimator/iekf.h"
#include "lvi_slam/map/voxel_map.h"

namespace lvi {

class LioUpdater : public ObservationModel {
 public:
  LioUpdater(const Config& cfg, VoxelMap* map);

  // 设置本轮要处理的点云（body 系，已去畸变、已降采样）
  void setInputCloud(const CloudPtr& cloud_body);

  bool buildResidual(const State& state, Residuals* out) override;
  const char* name() const override { return "LIO"; }

  // 更新完成后把点云灌进地图（带世界系协方差）
  void updateMap(const State& state);

  const std::vector<PointWithVar>& pointsWithVar() const { return pv_list_; }
  int lastValidCount() const { return last_valid_count_; }

 private:
  // 把 body 系协方差传播到世界系：
  //   Σ_w = R Σ_b R^T + [p]_x Σ_rot [p]_x^T + Σ_pos
  // 三项分别是测量噪声、姿态不确定度经叉乘传播、位置不确定度
  void propagatePointCov(const State& state);

  Config    cfg_;
  VoxelMap* map_ = nullptr;
  CloudPtr  cloud_body_;

  std::vector<PointWithVar> pv_list_;
  std::vector<M3D>          body_cov_list_;
  std::vector<M3D>          cross_mat_list_;
  std::vector<PointToPlane> ptpl_list_;
  int last_valid_count_ = 0;
};

}  // namespace lvi
