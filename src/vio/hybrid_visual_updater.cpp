// lvi_slam/vio/hybrid_visual_updater.cpp
// 混合视觉前端调度器。
//
// 设计动机：FAST-LIVO2 纯直接法，光照突变会崩；VINS-Mono 纯特征点法，
// 弱纹理会崩。两者失效场景几乎不重叠，双轨并行 + 健康度切换能覆盖更广的场景。
#include "lvi_slam/vio/vio_updater.h"

namespace lvi {

HybridVisualUpdater::HybridVisualUpdater(const Config& cfg, VoxelMap* map,
                                         CameraModel* cam)
    : cfg_(cfg) {
  direct_ = std::make_unique<DirectVisualUpdater>(cfg, map, cam);
  feature_ = std::make_unique<FeatureVisualUpdater>(cfg, map, cam);
}

// 返回本帧应该用哪个观测模型。返回 nullptr 表示两轨都不可用，
// 本帧放弃视觉更新 —— 纯靠 LIO，这比强行用一个坏观测去污染状态要好。
ObservationModel* HybridVisualUpdater::prepare(
    const cv::Mat& gray, double timestamp, const State& state,
    const std::vector<PointWithVar>& lidar_points) {

  direct_->setFrame(gray, timestamp);
  direct_->retrieveSubMap(state, lidar_points);

  // 特征点轨低频运行：正常时只为回环积累描述子，切换时才承担位姿估计。
  // 每 N 帧跑一次，或者已经处于备轨状态时每帧都跑。
  ++frame_count_;
  const bool need_feature = !cfg_.visual.hybrid_mode
                            || feature_track_countdown_ > 0
                            || (frame_count_ % kDescriptorInterval == 0);
  if (need_feature) {
    feature_->setFrame(gray, timestamp);
    feature_->associateDepthFromLidar(state, lidar_points);
  }

  if (!cfg_.visual.hybrid_mode) { track_ = Track::kDirect; return direct_.get(); }

  // 备轨倒计时：切过去之后维持若干帧再切回，防止在阈值附近来回震荡。
  // 光照恢复是渐变的，一切回去就又超阈值，会导致每帧都在切。
  if (feature_track_countdown_ > 0) {
    --feature_track_countdown_;
    track_ = Track::kFeature;
    return feature_.get();
  }

  track_ = Track::kDirect;
  return direct_.get();
}

// 更新完成后调用。健康度只能在 buildResidual 跑完后才知道，
// 所以切换决策在这里做，作用于下一帧。
void HybridVisualUpdater::finalize(const State& state) {
  const auto& health = direct_->health();

  if (track_ == Track::kDirect) {
    if (!health.is_healthy) {
      feature_track_countdown_ = kSwitchHoldFrames;
    } else {
      // 只有直接法在跑且健康时才更新视觉地图，避免两轨互相污染
      direct_->updateObservations(state);
      direct_->selectReferencePatches(state);
    }
  }
}

}  // namespace lvi
