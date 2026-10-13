// lvi_slam/frontend/frontend.h
// 前端：紧耦合 iEKF 里程计。
//
// 职责边界（这是整个架构最重要的一条线）：
//   前端只负责局部精度，维护一个带滑窗、内存有界的局部体素地图。
//   全局一致性完全交给后端——前端不做回环，不维护全局地图。
//
//   全局地图由后端的"关键帧位姿 + 关键帧点云"隐式定义，按需拼接。
//   这样回环优化改了位姿，全局地图自动就对了（LIO-SAM 模式），
//   避开了 FAST-LIO2 外接位姿图时"ikd-Tree 里存的是旧位姿变换过的点，改不了"
//   那个结构性问题。
#pragma once

#include "lvi_slam/common/types.h"
#include "lvi_slam/common/config.h"
#include "lvi_slam/common/diagnostics.h"
#include "lvi_slam/imu/imu_processor.h"
#include "lvi_slam/map/voxel_map.h"
#include "lvi_slam/estimator/iekf.h"
#include "lvi_slam/lio/lio_updater.h"
#include "lvi_slam/vio/vio_updater.h"
#include <deque>
#include <functional>

namespace lvi {

class Frontend {
 public:
  explicit Frontend(const Config& cfg);
  ~Frontend();

  // 处理一个同步包，内部完成 IMU 传播 + LIO/VIO 更新 + 地图更新
  bool process(const SyncPackage& pkg);

  const State& state() const { return state_; }
  bool initialized() const;

  // 关键帧产出回调，后端订阅
  using KeyframeCallback = std::function<void(const KeyframePtr&)>;
  void setKeyframeCallback(KeyframeCallback cb) { kf_callback_ = std::move(cb); }

  // 后端回环优化后调用（必须在前端主循环线程里调用，与 process 串行）：
  //   1. 用修正后的关键帧位姿重建前端局部地图
  //   2. 由关键帧 anchor_kf_id 的「前端位姿 → 优化后位姿」求出修正变换，
  //      作用到当前 iEKF 状态上
  // 不能直接把状态设成那个关键帧的优化位姿：后端处理有延迟，
  // anchor 关键帧可能已经是几帧之前的了，直接覆盖会让状态跳回过去。
  // 这是前后端信息同步的唯一入口。
  void applyLoopCorrection(const std::vector<KeyframePtr>& corrected_kfs,
                           int anchor_kf_id,
                           const M3D& anchor_rot, const V3D& anchor_pos);

  // 关键帧在当前前端坐标系下的位姿（回环修正后会随之更新）。
  // 只保留最近 kKfPoseHistory 个，查不到返回 false。
  int  lastKeyframeId() const { return kf_id_counter_ - 1; }
  bool keyframePose(int id, M3D* rot, V3D* pos) const;

  // 高频位姿输出（IMU 频率），供外部控制器使用
  State propagateHighFreq(const ImuData& imu);

  // 退化状态查询，用于日志和上层决策
  const IEKF::DegeneracyInfo& degeneracy() const { return degen_; }

  // 最近一次 process 的诊断记录（mode == 0 表示该帧还在初始化，没有更新）
  const FrameDiag& lastDiag() const { return diag_; }

 private:
  bool handleLio(const SyncPackage& pkg);
  bool handleVio(const SyncPackage& pkg);

  // 关键帧判据：距离 OR 角度 OR 时间兜底，再加协方差质量门槛。
  // 时间兜底这条容易被忽略——车停 30 秒不动，没有它就一个关键帧都没有，
  // 之后突然动起来位姿图会断裂。
  bool shouldCreateKeyframe() const;
  void createKeyframe(const SyncPackage& pkg);

  // LiDAR 退化时提高视觉权重，反之亦然。
  // 这是 LVI 相对 LIO/VIO 的核心价值，必须显式实现，
  // 不能指望"融合了自然就好"。
  void adaptCrossModalWeights();

  Config cfg_;
  State  state_;
  State  prior_;          // 本轮 IMU 传播得到的先验，iEKF 迭代中固定

  std::unique_ptr<ImuProcessor>        imu_;
  std::unique_ptr<VoxelMap>            map_;
  std::unique_ptr<IEKF>                iekf_;
  std::unique_ptr<LioUpdater>          lio_;
  std::unique_ptr<HybridVisualUpdater> vio_;
  std::unique_ptr<CameraModel>         cam_;

  CloudPtr cloud_undistort_;
  CloudPtr cloud_down_;

  IEKF::DegeneracyInfo degen_;

  // 关键帧管理
  KeyframeCallback kf_callback_;
  int    kf_id_counter_ = 0;
  M3D    last_kf_rot_ = M3D::Identity();
  V3D    last_kf_pos_ = V3D::Zero();
  double last_kf_time_ = -1.0;
  V3D    last_loop_kf_pos_ = V3D::Zero();

  struct KfPose { int id; M3D rot; V3D pos; };
  static constexpr size_t kKfPoseHistory = 2000;
  std::deque<KfPose> kf_poses_;

  FrameDiag diag_;
  void fillDiag(const SyncPackage& pkg, bool ok);

  // 外参在线标定的分阶段解锁
  double system_start_time_ = -1.0;
  bool   ext_rot_unlocked_ = false;
  bool   ext_pos_unlocked_ = false;
  void   updateExtrinsicUnlock();

  bool   map_initialized_ = false;
};

}  // namespace lvi
