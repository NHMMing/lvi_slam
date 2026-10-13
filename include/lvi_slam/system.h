// lvi_slam/system.h
// 系统顶层：把前端、后端、同步器串起来。
//
// 线程模型（刻意保持简单）：
//   主线程         : 传感器回调 + 前端主循环（串行，好调试）
//   后端优化线程   : 因子图增量优化
//   回环检测线程   : Scan Context 检索 + ICP 验证（低频，与优化解耦）
//   高频输出线程   : ROS Timer，IMU 频率的位姿前推
//
// 前端刻意不并行。FAST-LIVO2 单主循环最简洁，VINS-Mono 三节点三线程最复杂，
// 而两者精度在同类硬件上是可比的。过早并行会让调试成本指数上升。
// 等第一版跑通、profile 出真实瓶颈后再并行也不迟。
#pragma once

#include "lvi_slam/common/config.h"
#include "lvi_slam/sync/data_synchronizer.h"
#include "lvi_slam/frontend/frontend.h"
#include "lvi_slam/backend/backend.h"
#include <pcl/io/pcd_io.h>
#include <thread>
#include <atomic>

namespace lvi {


class System {
 public:
  explicit System(const Config& cfg);
  ~System();

  bool start();    // 启动前端主循环线程与后端线程
  void stop();

  // 传感器入口（供 ROS/ROS2 包装层调用）
  void feedImu(const ImuData& imu);
  void feedCloud(const CloudPtr& cloud, double t);
  void feedImage(const cv::Mat& img, double t);
  void feedGps(double t, const V3D& pos_enu, const V3D& cov);

  // 每次前端更新完成后回调，供 ROS 层发布高频位姿
  using OdometryCallback = std::function<void(const State&, double)>;
  void setOdometryCallback(OdometryCallback cb) { odom_callback_ = std::move(cb); }

  // 输出
  State currentState() const;

  // 积压的点云帧数（缓冲中的 + 前端正在处理的那一帧）。
  // 实时运行时不需要关心；离线全速回放必须据此限速，
  // 否则同步器缓冲满了会静默丢帧。
  size_t pendingClouds() const { return sync_->pendingClouds() + (busy_ ? 1 : 0); }
  CloudPtr buildGlobalMap(double leaf = 0.2) const;
  bool saveMapPCD(const std::string& path, double leaf = 0.1) const;

  // 离线收尾：等后端把队列里的关键帧全部入图，停线程，再做一次完整优化。
  // 之后 save*() 拿到的才是最终结果。在线节点退出时不需要调用。
  void finish();

  // ---- 评估输出（TUM 格式：t x y z qx qy qz qw，IMU/body 系位姿）----
  // 三条轨迹回答不同的问题：
  //   odometry   前端逐帧在线输出，因果、无「事后修正」，回环时会出现跳变——
  //              这是下游控制器实际拿到的位姿
  //   slam       同样逐帧，但每帧挂到所属关键帧上、用后端最终优化位姿重新表达——
  //              离线建图意义下的最优轨迹，和 odometry 对比就是后端/回环的收益
  //   keyframes  后端关键帧最终位姿（稀疏）
  bool saveTrajectoryTUM(const std::string& path) const;   // = odometry
  bool saveSlamTrajectoryTUM(const std::string& path) const;
  bool saveKeyframesTUM(const std::string& path) const;

  // 诊断记录（默认关闭，开销是每帧几百字节）。必须在 start() 之前调用。
  void enableDiagnostics(bool on);
  bool saveFrameDiagCSV(const std::string& path) const;
  bool saveLoopEventsCSV(const std::string& path) const;
  DataSynchronizer::DropStats dropStats() const { return sync_->dropStats(); }

 private:
  void mainLoop();
  // 后端线程产出的回环修正，暂存后由前端线程在两帧之间执行 ——
  // 前端状态和局部地图只允许前端线程碰。
  void applyPendingCorrection();

  Config cfg_;
  std::unique_ptr<DataSynchronizer> sync_;
  std::unique_ptr<Frontend>         frontend_;
  std::unique_ptr<Backend>          backend_;

  OdometryCallback  odom_callback_;
  std::thread       main_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> busy_{false};      // 前端正在处理一帧

  struct PendingCorrection {
    std::vector<KeyframePtr> kfs;
    int anchor_id = -1;
    M3D rot;
    V3D pos;
  };
  std::mutex correction_mutex_;
  std::unique_ptr<PendingCorrection> pending_correction_;

  // 逐帧记录：前端位姿 + 所属关键帧及相对位姿（用于生成 slam 轨迹）
  struct FrameRecord {
    double t;
    Eigen::Matrix4d T_odom;
    int anchor_kf = -1;
    Eigen::Matrix4d T_anchor_frame;   // T_anchor⁻¹ · T_frame
  };
  mutable std::mutex traj_mutex_;
  std::vector<FrameRecord> trajectory_;

  bool diag_enabled_ = false;
  std::vector<FrameDiag> frame_diags_;   // 受 traj_mutex_ 保护
};

}  // namespace lvi
