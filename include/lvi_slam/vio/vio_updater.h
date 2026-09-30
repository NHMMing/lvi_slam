// lvi_slam/vio/vio_updater.h
// 视觉观测模型：混合前端（直接法主轨 + 特征点法备轨）。
//
// 为什么要混合：
//   FAST-LIVO2 纯直接法，光照突变（进隧道、逆光、闪光）时亮度恒定假设失效，
//   逆曝光时间只能补全局亮度，补不了局部光照变化。
//   VINS-Mono 纯特征点法，弱纹理时提不出足够角点。
//
//   两者的失效场景不重叠，所以并行跑两轨、按健康度切换是有意义的。
//
// 特征点轨还有一个不可替代的用途：直接法没有描述子，做不了回环检索。
//   所以即使直接法一切正常，特征点轨也要低频运行，为后端积累描述子。
#pragma once

#include "lvi_slam/common/types.h"
#include "lvi_slam/common/config.h"
#include "lvi_slam/estimator/iekf.h"
#include "lvi_slam/map/voxel_map.h"
#include "lvi_slam/vio/visual_types.h"
#include "lvi_slam/vio/camera_model.h"

namespace lvi {

// ---------------------------------------------------------------------------
// 直接法观测模型
// ---------------------------------------------------------------------------
class DirectVisualUpdater : public ObservationModel {
 public:
  DirectVisualUpdater(const Config& cfg, VoxelMap* map, CameraModel* cam);

  // 每帧调用顺序：
  //   setFrame -> retrieveSubMap -> [iEKF.update] -> generateNewPoints
  //            -> updateObservations -> selectReferencePatches
  void setFrame(const cv::Mat& gray, double timestamp);

  // 从体素地图检索本帧可见的视觉点，做仿射变换准备参考 patch
  void retrieveSubMap(const State& state,
                      const std::vector<PointWithVar>& lidar_points);

  bool buildResidual(const State& state, Residuals* out) override;
  const char* name() const override { return "VIO-Direct"; }

  // 用本帧 LiDAR 点在无视觉点的网格中补充新的视觉点
  void generateNewPoints(const State& state,
                         const std::vector<PointWithVar>& lidar_points);


  // 把当前帧的观测加入各视觉点的 obs 列表
  void updateObservations(const State& state);

  // 参考 patch 优选 + 法向量在线精化。
  // 打分 = 与其他观测的平均 NCC + 观测角度余弦。
  // 前者衡量该 patch 是否"典型"（无遮挡、无模糊），后者衡量透视畸变大小。
  void selectReferencePatches(const State& state);

  const PhotometricHealth& health() const { return health_; }
  const VisualSubMap& subMap() const { return submap_; }

 private:
  // 精确平面单应做 patch 仿射变换：H = R - t*n^T/d
  // 法向量来自 LiDAR 拟合的平面，这是特征点法做不到的
  void warpAffineHomography(const V2D& px_ref, const V3D& xyz_ref,
                            const V3D& normal_ref,
                            const M3D& R_cur_ref, const V3D& t_cur_ref,
                            Eigen::Matrix2d* A_cur_ref) const;

  // 无法向量时的退化方案：假设 patch 内深度相同（fronto-parallel）
  void warpAffineDepth(const V2D& px_ref, const V3D& bearing_ref,
                       double depth_ref,
                       const M3D& R_cur_ref, const V3D& t_cur_ref,
                       Eigen::Matrix2d* A_cur_ref) const;

  // 按仿射行列式选金字塔层，避免放大/缩小时的采样混叠
  int  bestSearchLevel(const Eigen::Matrix2d& A_cur_ref, int max_level) const;

  void computeHealth();

  Config       cfg_;
  VoxelMap*    map_ = nullptr;
  CameraModel* cam_ = nullptr;

  cv::Mat      img_gray_;
  std::vector<cv::Mat> pyramid_;
  double       timestamp_ = 0.0;

  VisualSubMap submap_;
  std::vector<int> grid_occupied_;      // 网格占用标记，保证特征均匀分布
  std::vector<VisualPointPtr> grid_best_point_;
  std::vector<float> grid_best_dist_;

  PhotometricHealth  health_;
  std::deque<double> residual_history_;  // 光度残差滑动窗口，健康度基线
};

// ---------------------------------------------------------------------------
// 特征点法观测模型（备轨 + 回环描述子来源）
// ---------------------------------------------------------------------------
class FeatureVisualUpdater : public ObservationModel {
 public:
  FeatureVisualUpdater(const Config& cfg, VoxelMap* map, CameraModel* cam);

  void setFrame(const cv::Mat& gray, double timestamp);

  // 深度直接来自 LiDAR，不需要三角化。
  // 这是 LVI 系统相对纯 VIO 的天然优势——省掉了整个三角化和逆深度优化。
  void associateDepthFromLidar(const State& state,
                               const std::vector<PointWithVar>& lidar_points);

  bool buildResidual(const State& state, Residuals* out) override;
  const char* name() const override { return "VIO-Feature"; }

  // 输出给后端做回环检索
  cv::Mat descriptors() const { return descriptors_; }
  const std::vector<cv::KeyPoint>& keypoints() const { return keypoints_; }

 private:
  Config       cfg_;
  VoxelMap*    map_ = nullptr;
  CameraModel* cam_ = nullptr;

  cv::Mat      img_gray_;
  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat      descriptors_;
  std::vector<V3D>  points_3d_;    // 与 keypoints_ 一一对应，深度来自 LiDAR
  std::vector<bool> has_depth_;
  cv::Ptr<cv::ORB>  orb_;
  double            timestamp_ = 0.0;
};

// ---------------------------------------------------------------------------
// 混合调度器：按光度健康度在两轨间切换
// ---------------------------------------------------------------------------
class HybridVisualUpdater {
 public:
  HybridVisualUpdater(const Config& cfg, VoxelMap* map, CameraModel* cam);

  enum class Track { kDirect, kFeature };

  // 返回本轮实际使用的观测模型；调用方拿去喂给 iEKF
  ObservationModel* prepare(const cv::Mat& gray, double timestamp,
                            const State& state,
                            const std::vector<PointWithVar>& lidar_points);

  // iEKF 更新完成后的收尾：更新地图点、选参考 patch、提描述子
  void finalize(const State& state);

  Track currentTrack() const { return track_; }
  cv::Mat loopDescriptors() const { return feature_->descriptors(); }

 private:
  Config cfg_;
  std::unique_ptr<DirectVisualUpdater>  direct_;
  std::unique_ptr<FeatureVisualUpdater> feature_;
  Track track_ = Track::kDirect;
  int   feature_track_countdown_ = 0;   // 切到备轨后维持若干帧再切回，防抖
  uint64_t frame_count_ = 0;

  static constexpr int kSwitchHoldFrames    = 10;  // 切备轨后维持帧数
  static constexpr int kDescriptorInterval  = 5;   // 正常时每 N 帧提一次描述子
};

}  // namespace lvi
