// lvi_slam/system.cpp
// 系统顶层：把同步器、前端、后端串起来。
//
// 线程模型（刻意保持简单）：
//   传感器回调线程 —— ROS 层调用 feedXxx，只做入队
//   main_thread_   —— 前端主循环，串行跑 同步 → iEKF → 地图更新
//   后端两个线程   —— 因子图优化 + 回环检测（Backend 内部管理）
//
// 前端刻意不并行。FAST-LIVO2 单主循环最简洁，VINS-Mono 三节点三线程最复杂，
// 两者精度在同类硬件上可比。过早并行会让调试成本指数上升 ——
// 等跑通、profile 出真实瓶颈再并行也不迟。
#include "lvi_slam/system.h"
#include <fstream>
#include <iomanip>

namespace lvi {

System::System(const Config& cfg) : cfg_(cfg) {
  sync_ = std::make_unique<DataSynchronizer>(cfg);
  frontend_ = std::make_unique<Frontend>(cfg);
  backend_ = std::make_unique<Backend>(cfg);

  // 前端产出关键帧 → 后端消费
  frontend_->setKeyframeCallback(
      [this](const KeyframePtr& kf) { backend_->addKeyframe(kf); });

  // 后端回环优化完成 → 前端重建局部地图 + 重置 iEKF 状态。
  // 这是前后端同步的唯一入口。
  backend_->setLoopCallback(
      [this](const std::vector<KeyframePtr>& kfs, const M3D& R, const V3D& p) {
        frontend_->applyLoopCorrection(kfs, R, p);
      });
}

System::~System() { stop(); }

bool System::start() {
  if (running_) return true;
  running_ = true;
  if (cfg_.backend.enable) backend_->start();
  main_thread_ = std::thread(&System::mainLoop, this);
  return true;
}

void System::stop() {
  if (!running_) return;
  running_ = false;
  if (main_thread_.joinable()) main_thread_.join();
  backend_->stop();
}

void System::feedImu(const ImuData& imu) {
  sync_->pushImu(imu);
  // 高频位姿前推：不等前端更新，直接用最新状态积分。
  // 延迟极低（IMU 同频），精度不如前端输出，供外部控制回路用。
  if (odom_callback_ && frontend_->initialized()) {
    odom_callback_(frontend_->propagateHighFreq(imu), imu.t);
  }
}

void System::feedCloud(const CloudPtr& cloud, double t) { sync_->pushCloud(cloud, t); }
void System::feedImage(const cv::Mat& img, double t)    { sync_->pushImage(img, t); }
void System::feedGps(double t, const V3D& pos, const V3D& cov) {
  backend_->addGps(t, pos, cov);
}

// 前端主循环。整个状态估计是严格串行的：同步 → IMU传播 → LIO/VIO更新 → 地图更新。
void System::mainLoop() {
  SyncPackage pkg;
  while (running_) {
    if (!sync_->trySync(&pkg)) {
      // 没有数据就短睡，不要忙等 —— 忙等会把一个核吃满，
      // 在嵌入式平台上直接影响其他线程。
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }
    if (!frontend_->process(pkg)) continue;

    const State& s = frontend_->state();
    {
      std::lock_guard<std::mutex> lk(traj_mutex_);
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T.block<3, 3>(0, 0) = s.rot;
      T.block<3, 1>(0, 3) = s.pos;
      trajectory_.emplace_back(s.timestamp, T);
    }
    // 前端更新后的位姿精度高于 IMU 前推，覆盖发布
    if (odom_callback_) odom_callback_(s, s.timestamp);
  }
}

State System::currentState() const { return frontend_->state(); }

CloudPtr System::buildGlobalMap(double leaf) const {
  return backend_->buildGlobalMap(leaf);
}

// TUM 格式：timestamp tx ty tz qx qy qz qw
// 直接喂给 evo_ape / evo_rpe，不需要任何转换。
bool System::saveTrajectoryTUM(const std::string& path) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  ofs << std::fixed << std::setprecision(9);

  std::lock_guard<std::mutex> lk(traj_mutex_);
  for (const auto& [t, T] : trajectory_) {
    const Eigen::Quaterniond q(Eigen::Matrix3d(T.block<3, 3>(0, 0)));
    ofs << t << " "
        << T(0, 3) << " " << T(1, 3) << " " << T(2, 3) << " "
        << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  return true;
}

bool System::saveMapPCD(const std::string& path, double leaf) const {
  CloudPtr map = buildGlobalMap(leaf);
  if (!map || map->empty()) return false;
  return pcl::io::savePCDFileBinary(path, *map) == 0;
}

}  // namespace lvi
