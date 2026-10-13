// lvi_slam/backend/backend.h
// 后端：因子图 + 回环检测。
//
// 为什么前端用滤波、后端用因子图：
//   滤波在处理海量观测时效率碾压优化（对偶形式让计算量与观测数解耦），
//   所以局部里程计用滤波。
//   但回环和多源融合（GPS/UWB/轮速）在因子图下是"加一条边"的事，
//   在滤波框架下是架构性难题，所以全局一致性用因子图。
//
// 地图组织借鉴 LIO-SAM：地图 = 关键帧位姿 + 关键帧点云，按需拼接。
// 位姿被优化改了，地图自动就对了——这是 FAST-LIO2 外接位姿图方案
// 解决不了的问题（ikd-Tree 里存的是已变换到世界系的点）。
#pragma once

#include "lvi_slam/common/types.h"
#include "lvi_slam/common/config.h"
#include "lvi_slam/backend/loop_detector.h"
#include "lvi_slam/common/diagnostics.h"
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>

// GTSAM 前向声明，避免头文件污染
namespace gtsam {
class ISAM2;
class NonlinearFactorGraph;
class Values;
}

namespace lvi {

class Backend {
 public:
  explicit Backend(const Config& cfg);
  ~Backend();

  void start();
  void stop();

  // 前端产出关键帧时调用（线程安全，内部入队）
  void addKeyframe(const KeyframePtr& kf);

  // GPS 数据（可选）
  void addGps(double t, const V3D& pos_enu, const V3D& cov_diag);

  // 回环优化完成后的回调，前端订阅以重建局部地图。
  // 参数：最近若干关键帧的位姿快照（独立副本，点云共享），
  //       以及作为修正基准的关键帧 id 和它优化后的位姿。
  // 在后端线程里调用；接收方负责转交给前端线程执行。
  using LoopCallback = std::function<void(const std::vector<KeyframePtr>&,
                                          int, const M3D&, const V3D&)>;
  void setLoopCallback(LoopCallback cb) { loop_callback_ = std::move(cb); }

  // 阻塞直到队列中的关键帧全部入图（离线评估收尾用），超时返回 false
  bool flush(double timeout_sec = 30.0);
  // stop() 之后调用：把尚未入图的回环约束加进去，再做一次完整优化并写回全部关键帧位姿。
  // 平时只有回环发生时才写回，两次回环之间旧关键帧的小幅修正不会体现在 keyframes() 里。
  void finalize();

  // 回环检测诊断事件（线程安全拷贝）
  void enableDiagnostics(bool on) { diag_enabled_ = on; }
  std::vector<LoopEvent> loopEvents() const;

  // 拼接全局地图（按需，不常驻内存）
  CloudPtr buildGlobalMap(double leaf_size = 0.2) const;

  std::vector<KeyframePtr> keyframes() const;

 private:
  void optimizeThread();     // 主优化线程：处理关键帧队列，增量优化
  void loopThread();         // 回环检测线程：低频运行，与优化解耦

  // ---- 因子构建 ----
  void addOdomFactor(const KeyframePtr& kf);
  void addGpsFactor(const KeyframePtr& kf);
  void addLoopFactors();

  // 回环优化后把新位姿写回所有关键帧
  void correctPoses();

  Config cfg_;

  std::unique_ptr<gtsam::ISAM2>                isam_;
  std::unique_ptr<gtsam::NonlinearFactorGraph> graph_;
  std::unique_ptr<gtsam::Values>               initial_;
  std::unique_ptr<LoopDetector>                loop_detector_;

  mutable std::mutex kf_mutex_;
  std::vector<KeyframePtr> keyframes_;
  std::deque<KeyframePtr>  pending_kfs_;
  std::condition_variable  kf_cv_;

  std::mutex loop_mutex_;
  std::vector<LoopConstraint> pending_loops_;

  struct GpsData { double t; V3D pos; V3D cov; };
  std::mutex gps_mutex_;
  std::deque<GpsData> gps_buf_;
  V3D last_gps_pos_ = V3D::Zero();
  bool has_last_gps_ = false;

  Eigen::Matrix<double, 6, 6> latest_pose_cov_ =
      Eigen::Matrix<double, 6, 6>::Identity();

  LoopCallback loop_callback_;

  std::thread opt_thread_, loop_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> loop_closed_{false};
  std::atomic<bool> optimizing_{false};   // 已从队列取出、尚未入图的关键帧

  std::atomic<bool> diag_enabled_{false};
  mutable std::mutex event_mutex_;
  std::vector<LoopEvent> loop_events_;
  void recordLoopEvent(const LoopConstraint& lc, LoopStage stage,
                       const std::vector<KeyframePtr>& kfs);
};

}  // namespace lvi
