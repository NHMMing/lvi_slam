// lvi_slam/backend/backend.cpp
// 后端：GTSAM 因子图 + ISAM2 增量优化。
//
// 分工：前端 iEKF 负责局部精度，后端因子图负责全局一致性。
// 回环在滤波框架下无处安放（EKF 是马尔可夫的，回不去修历史），
// 这是本系统相对 FAST-LIVO2 的核心增量。
//
// 线程模型：
//   optimizeThread — 消费关键帧队列，构建因子，ISAM2 增量优化
//   loopThread     — 低频跑回环检测（ICP 很慢，不能阻塞优化）
// 两者通过 pending_loops_ 解耦。
#include "lvi_slam/backend/backend.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/filters/voxel_grid.h>

#include <chrono>

namespace lvi {
namespace {

using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::symbol_shorthand::X;

inline Pose3 toPose3(const M3D& R, const V3D& t) {
  return Pose3(Rot3(R), Point3(t));
}

inline void fromPose3(const Pose3& p, M3D* R, V3D* t) {
  *R = p.rotation().matrix();
  *t = p.translation();
}

}  // namespace

Backend::Backend(const Config& cfg) : cfg_(cfg) {
  gtsam::ISAM2Params params;
  // relinearizeThreshold 决定变量变化多大才重新线性化。
  // 设小了每次都重线性化（慢），设大了线性化点过期（不准）。0.1 是经验平衡点。
  params.relinearizeThreshold = 0.1;
  params.relinearizeSkip = 1;
  isam_ = std::make_unique<gtsam::ISAM2>(params);
  graph_ = std::make_unique<gtsam::NonlinearFactorGraph>();
  initial_ = std::make_unique<gtsam::Values>();
  loop_detector_ = std::make_unique<LoopDetector>(cfg);
}

Backend::~Backend() { stop(); }

void Backend::start() {
  if (running_) return;
  running_ = true;
  opt_thread_ = std::thread(&Backend::optimizeThread, this);
  if (cfg_.backend.enable) {
    loop_thread_ = std::thread(&Backend::loopThread, this);
  }
}

void Backend::stop() {
  if (!running_) return;
  running_ = false;
  kf_cv_.notify_all();
  if (opt_thread_.joinable()) opt_thread_.join();
  if (loop_thread_.joinable()) loop_thread_.join();
}

void Backend::addKeyframe(const KeyframePtr& kf) {
  {
    std::lock_guard<std::mutex> lk(kf_mutex_);
    pending_kfs_.push_back(kf);
  }
  kf_cv_.notify_one();
}

void Backend::addGps(double t, const V3D& pos_enu, const V3D& cov_diag) {
  std::lock_guard<std::mutex> lk(gps_mutex_);
  gps_buf_.push_back({t, pos_enu, cov_diag});
  // 缓冲区有上限，防止 GPS 一直来但从未被消费导致内存增长
  while (gps_buf_.size() > 2000) gps_buf_.pop_front();
}

// ---------------------------------------------------------------------------
// 里程计因子
//
// 与 LIO-SAM 的关键区别：噪声不是写死的常数，而是用前端给出的位姿协方差。
// 前端知道自己这一段跑得准不准（退化时协方差会涨），把这个信息传到后端，
// 比拍脑袋定一个 1e-6 合理得多。
// ---------------------------------------------------------------------------
void Backend::addOdomFactor(const KeyframePtr& kf) {
  const size_t id = keyframes_.size();

  if (id == 0) {
    // 第一帧先验。这一行噪声设置直接编码了 LVI 系统的可观性分析结论：
    //   roll/pitch 强约束 —— IMU 感知重力方向，绝对可观，不漂移
    //   yaw + xyz 几乎无约束 —— 这就是那 4 个不可观自由度
    // 不这么设的话，因子图会把初始位姿当成绝对真值锁死，回环时拧不动。
    auto prior_noise = gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished());
    graph_->add(gtsam::PriorFactor<Pose3>(X(0), toPose3(kf->rot, kf->pos), prior_noise));
    initial_->insert(X(0), toPose3(kf->rot, kf->pos));
    return;
  }

  const auto& prev = keyframes_.back();
  const Pose3 pose_prev = toPose3(prev->rot, prev->pos);
  const Pose3 pose_curr = toPose3(kf->rot, kf->pos);

  // 前端协方差 → 因子噪声。加下限防止过度自信（协方差接近 0 会让这条边变成硬约束，
  // 回环时整条轨迹拧不动）。
  gtsam::Vector6 var;
  for (int i = 0; i < 6; ++i) {
    var(i) = std::max(kf->pose_cov(i, i), 1e-6);
  }
  auto odom_noise = gtsam::noiseModel::Diagonal::Variances(var);

  graph_->add(gtsam::BetweenFactor<Pose3>(X(id - 1), X(id),
                                          pose_prev.between(pose_curr), odom_noise));
  initial_->insert(X(id), pose_curr);
}

// ---------------------------------------------------------------------------
// GPS 因子。多重过滤，设计哲学是「GPS 只在激光里程计不确定时才介入」。
// 直接把每个 GPS 都加进去会让噪声污染本来很准的里程计。
// ---------------------------------------------------------------------------
void Backend::addGpsFactor(const KeyframePtr& kf) {
  if (!cfg_.backend.use_gps) return;

  std::lock_guard<std::mutex> lk(gps_mutex_);
  if (gps_buf_.empty() || keyframes_.empty()) return;

  // 门槛 1：走够距离才用 GPS。开局原地不动时航向未收敛，GPS 会把系统带偏。
  if ((keyframes_.back()->pos - keyframes_.front()->pos).norm() < 5.0) return;

  // 门槛 2：当前位姿不确定度足够小就不需要 GPS。这是最关键的一条 ——
  // 它让 GPS 成为「漂移兜底」而不是「常规观测」。
  if (latest_pose_cov_(3, 3) < cfg_.backend.pose_cov_thresh &&
      latest_pose_cov_(4, 4) < cfg_.backend.pose_cov_thresh) {
    return;
  }

  // 时间对齐：丢掉太旧的，等太新的
  while (!gps_buf_.empty() && gps_buf_.front().t < kf->t - 0.2) gps_buf_.pop_front();
  if (gps_buf_.empty() || gps_buf_.front().t > kf->t + 0.2) return;

  const GpsData g = gps_buf_.front();
  gps_buf_.pop_front();

  // 门槛 3：GPS 自身质量
  if (g.cov.x() > cfg_.backend.gps_cov_thresh ||
      g.cov.y() > cfg_.backend.gps_cov_thresh) return;
  // 门槛 4：未定位的 (0,0,0)
  if (g.pos.head<2>().norm() < 1e-6) return;
  // 门槛 5：空间间隔，避免因子过密
  if (has_last_gps_ && (g.pos - last_gps_pos_).norm() < cfg_.backend.gps_min_stride) return;

  last_gps_pos_ = g.pos;
  has_last_gps_ = true;

  // 高度默认不用 GPS —— GNSS 的垂直精度通常比水平差 2-3 倍，
  // 而激光的 z 通常很准（地面约束强）。
  V3D pos = g.pos;
  V3D cov = g.cov;
  if (!cfg_.backend.use_gps_z) {
    pos.z() = kf->pos.z();
    cov.z() = 0.01;
  }
  gtsam::Vector3 v(std::max(cov.x(), 1.0), std::max(cov.y(), 1.0),
                   std::max(cov.z(), 0.01));
  graph_->add(gtsam::GPSFactor(X(keyframes_.size()), Point3(pos),
                               gtsam::noiseModel::Diagonal::Variances(v)));
}

// ---------------------------------------------------------------------------
// 回环因子。
//
// 噪声用 ICP 的 fitness score 自适应 —— 匹配越好，噪声越小，权重越大。
// 外面再套一层 Cauchy 鲁棒核：即使前面的双模态验证漏过了一个假回环，
// 鲁棒核也能限制它对整张图的破坏。误回环是因子图的头号杀手，
// 一条错误的边能把整条轨迹拧坏，多一层保险很值。
// ---------------------------------------------------------------------------
void Backend::addLoopFactors() {
  std::vector<LoopConstraint> loops;
  {
    std::lock_guard<std::mutex> lk(loop_mutex_);
    if (pending_loops_.empty()) return;
    loops.swap(pending_loops_);
  }

  for (const auto& lc : loops) {
    if (lc.id_from < 0 || lc.id_to < 0) continue;
    if (lc.id_from >= static_cast<int>(keyframes_.size()) ||
        lc.id_to >= static_cast<int>(keyframes_.size())) {
      continue;  // 关键帧还没入图，丢弃（下一轮检测会重新找到）
    }

    auto base = gtsam::noiseModel::Diagonal::Variances(
        gtsam::Vector6::Constant(std::max(lc.noise, 1e-4)));
    auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1.0), base);

    graph_->add(gtsam::BetweenFactor<Pose3>(X(lc.id_to), X(lc.id_from),
                                            toPose3(lc.rot_delta, lc.pos_delta),
                                            robust));
    loop_closed_ = true;
  }
}

// ---------------------------------------------------------------------------
// 主优化线程
// ---------------------------------------------------------------------------
void Backend::optimizeThread() {
  while (running_) {
    KeyframePtr kf;
    {
      std::unique_lock<std::mutex> lk(kf_mutex_);
      kf_cv_.wait(lk, [this] { return !pending_kfs_.empty() || !running_; });
      if (!running_) break;
      kf = pending_kfs_.front();
      pending_kfs_.pop_front();
    }
    if (!kf) continue;

    const size_t new_id = keyframes_.size();
    kf->id = static_cast<int>(new_id);

    addOdomFactor(kf);
    addGpsFactor(kf);
    addLoopFactors();

    isam_->update(*graph_, *initial_);
    isam_->update();
    if (loop_closed_) {
      // 回环会导致大量位姿被修正，一两次 update 不够收敛。
      // ISAM2 每次 update 只做局部重线性化，多跑几次代价可控。
      for (int i = 0; i < 5; ++i) isam_->update();
    }
    graph_->resize(0);
    initial_->clear();

    const gtsam::Values est = isam_->calculateEstimate();

    {
      std::lock_guard<std::mutex> lk(kf_mutex_);
      keyframes_.push_back(kf);
      fromPose3(est.at<Pose3>(X(new_id)), &kf->rot, &kf->pos);
    }
    latest_pose_cov_ = isam_->marginalCovariance(X(new_id));

    // 关键帧一旦入图就交给回环检索库。
    // 只有 is_loop_candidate 的才真正入库（前端已按空间间隔稀疏化过）。
    loop_detector_->addKeyframe(kf);

    if (loop_closed_) {
      correctPoses();
      loop_closed_ = false;

      // 通知前端：用修正后的位姿重建局部地图 + 重置 iEKF 状态。
      //
      // 这是前后端同步的唯一入口，也是本系统相对「FAST-LIO2 + 外接位姿图」
      // 的关键改进 —— 那个方案里 ikd-Tree 存的是已变换到世界系的点，
      // 回环改了位姿地图也改不了。这里地图由关键帧位姿隐式定义，
      // 位姿一改，重建一次局部地图就全对了。
      if (loop_callback_) {
        std::vector<KeyframePtr> recent;
        {
          std::lock_guard<std::mutex> lk(kf_mutex_);
          const int total = static_cast<int>(keyframes_.size());
          const int begin = std::max(0, total - cfg_.backend.relocal_kf_num);
          recent.assign(keyframes_.begin() + begin, keyframes_.end());
        }
        loop_callback_(recent, kf->rot, kf->pos);
      }
    }
  }
}

// 回环优化后把新位姿写回所有关键帧。
// 注意只改位姿，不动点云 —— 点云存的是 body 系，与位姿解耦。
// 这正是「地图 = 关键帧位姿 + 关键帧点云」这个设计的好处。
void Backend::correctPoses() {
  const gtsam::Values est = isam_->calculateEstimate();
  std::lock_guard<std::mutex> lk(kf_mutex_);
  for (size_t i = 0; i < keyframes_.size(); ++i) {
    if (!est.exists(X(i))) continue;
    fromPose3(est.at<Pose3>(X(i)), &keyframes_[i]->rot, &keyframes_[i]->pos);
  }
}

// ---------------------------------------------------------------------------
// 回环检测线程。低频运行，与优化线程完全解耦。
// ---------------------------------------------------------------------------
void Backend::loopThread() {
  using namespace std::chrono_literals;
  while (running_) {
    std::this_thread::sleep_for(500ms);   // 2Hz 足够，ICP 本来就慢

    KeyframePtr current;
    std::vector<KeyframePtr> snapshot;
    {
      std::lock_guard<std::mutex> lk(kf_mutex_);
      if (keyframes_.size() < 30) continue;
      snapshot = keyframes_;     // 浅拷贝指针，代价很小
      current = snapshot.back();
    }

    LoopConstraint lc;
    if (!loop_detector_->detect(current, snapshot, &lc)) continue;

    std::lock_guard<std::mutex> lk(loop_mutex_);
    pending_loops_.push_back(lc);
  }
}

// ---------------------------------------------------------------------------
// 全局地图按需拼接 —— 不常驻内存。
//
// 这是整个地图策略的落脚点：全局地图从来不是一个被维护的数据结构，
// 而是「关键帧位姿 + 关键帧 body 系点云」的一个函数。
// 位姿被回环改了，下次调用这个函数结果自动就是对的。
// ---------------------------------------------------------------------------
CloudPtr Backend::buildGlobalMap(double leaf_size) const {
  CloudPtr global(new PointCloud());
  std::vector<KeyframePtr> kfs = keyframes();

  for (const auto& kf : kfs) {
    if (!kf || !kf->cloud_body) continue;
    for (const auto& p : kf->cloud_body->points) {
      const V3D pw = kf->rot * V3D(p.x, p.y, p.z) + kf->pos;
      PointType q = p;
      q.x = static_cast<float>(pw.x());
      q.y = static_cast<float>(pw.y());
      q.z = static_cast<float>(pw.z());
      global->push_back(q);
    }
  }
  if (leaf_size > 0 && !global->empty()) {
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.setInputCloud(global);
    CloudPtr filtered(new PointCloud());
    vg.filter(*filtered);
    return filtered;
  }
  return global;
}

std::vector<KeyframePtr> Backend::keyframes() const {
  std::lock_guard<std::mutex> lk(kf_mutex_);
  return keyframes_;
}

}  // namespace lvi
