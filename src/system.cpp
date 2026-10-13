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
#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace lvi {

System::System(const Config& cfg) : cfg_(cfg) {
  sync_ = std::make_unique<DataSynchronizer>(cfg);
  frontend_ = std::make_unique<Frontend>(cfg);
  backend_ = std::make_unique<Backend>(cfg);

  // 前端产出关键帧 → 后端消费
  frontend_->setKeyframeCallback(
      [this](const KeyframePtr& kf) { backend_->addKeyframe(kf); });

  // 后端回环优化完成 → 前端重建局部地图 + 修正 iEKF 状态。
  // 这是前后端同步的唯一入口。回调发生在后端线程，而前端状态和局部地图
  // 正被主循环读写，所以这里只暂存，由主循环在两帧之间执行。
  // 连续多次回环只保留最新一次：它基于更完整的图，包含了之前的修正。
  backend_->setLoopCallback(
      [this](const std::vector<KeyframePtr>& kfs, int anchor_id,
             const M3D& R, const V3D& p) {
        auto c = std::make_unique<PendingCorrection>();
        c->kfs = kfs;
        c->anchor_id = anchor_id;
        c->rot = R;
        c->pos = p;
        std::lock_guard<std::mutex> lk(correction_mutex_);
        pending_correction_ = std::move(c);
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

void System::applyPendingCorrection() {
  std::unique_ptr<PendingCorrection> c;
  {
    std::lock_guard<std::mutex> lk(correction_mutex_);
    c = std::move(pending_correction_);
  }
  if (c) frontend_->applyLoopCorrection(c->kfs, c->anchor_id, c->rot, c->pos);
}

namespace {
Eigen::Matrix4d toMatrix(const M3D& R, const V3D& p) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = p;
  return T;
}
}  // namespace

// 前端主循环。整个状态估计是严格串行的：同步 → IMU传播 → LIO/VIO更新 → 地图更新。
void System::mainLoop() {
  SyncPackage pkg;
  while (running_) {
    applyPendingCorrection();

    // busy_ 在 trySync 之前置位：否则包已出缓冲、还没开始处理的瞬间，
    // pendingClouds() 会读到 0，离线回放会误以为前端已经处理完。
    busy_ = true;
    if (!sync_->trySync(&pkg)) {
      busy_ = false;
      // 没有数据就短睡，不要忙等 —— 忙等会把一个核吃满，
      // 在嵌入式平台上直接影响其他线程。
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }
    const auto tic = std::chrono::steady_clock::now();
    const bool ok = frontend_->process(pkg);
    const double proc_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - tic).count();

    if (diag_enabled_ && frontend_->lastDiag().mode != 0) {
      FrameDiag d = frontend_->lastDiag();
      d.proc_ms = proc_ms;
      std::lock_guard<std::mutex> lk(traj_mutex_);
      frame_diags_.push_back(d);
    }
    if (!ok) { busy_ = false; continue; }

    const State& s = frontend_->state();
    {
      FrameRecord r;
      r.t = s.timestamp;
      r.T_odom = toMatrix(s.rot, s.pos);
      r.anchor_kf = frontend_->lastKeyframeId();
      M3D R_kf;
      V3D p_kf;
      if (r.anchor_kf >= 0 && frontend_->keyframePose(r.anchor_kf, &R_kf, &p_kf)) {
        r.T_anchor_frame = toMatrix(R_kf, p_kf).inverse() * r.T_odom;
      } else {
        r.anchor_kf = -1;
      }
      std::lock_guard<std::mutex> lk(traj_mutex_);
      trajectory_.push_back(r);
    }
    // 前端更新后的位姿精度高于 IMU 前推，覆盖发布
    if (odom_callback_) odom_callback_(s, s.timestamp);
    busy_ = false;
  }
}

void System::finish() {
  // 前端已排空（调用方保证）；后端可能还有关键帧在排队
  if (!backend_->flush()) {
    std::cerr << "[System] backend flush timed out, final optimization may be incomplete\n";
  }
  stop();
  backend_->finalize();
}

void System::enableDiagnostics(bool on) {
  diag_enabled_ = on;
  backend_->enableDiagnostics(on);
}

State System::currentState() const { return frontend_->state(); }

CloudPtr System::buildGlobalMap(double leaf) const {
  return backend_->buildGlobalMap(leaf);
}

// TUM 格式：timestamp tx ty tz qx qy qz qw
// 直接喂给 tools/eval 或 evo，不需要任何转换。
namespace {
void writeTumLine(std::ofstream& ofs, double t, const M3D& R, const V3D& p) {
  const Eigen::Quaterniond q(R);
  ofs << t << " " << p.x() << " " << p.y() << " " << p.z() << " "
      << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
}
}  // namespace

bool System::saveTrajectoryTUM(const std::string& path) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  ofs << std::fixed << std::setprecision(9);

  std::lock_guard<std::mutex> lk(traj_mutex_);
  for (const auto& r : trajectory_) {
    writeTumLine(ofs, r.t, r.T_odom.block<3, 3>(0, 0), r.T_odom.block<3, 1>(0, 3));
  }
  return true;
}

bool System::saveSlamTrajectoryTUM(const std::string& path) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  ofs << std::fixed << std::setprecision(9);

  // 后端关键帧 id 与前端一致（都从 0 连续递增），可以直接下标
  const std::vector<KeyframePtr> kfs = backend_->keyframes();
  std::lock_guard<std::mutex> lk(traj_mutex_);
  for (const auto& r : trajectory_) {
    if (r.anchor_kf < 0 || r.anchor_kf >= static_cast<int>(kfs.size())) continue;
    const auto& kf = kfs[r.anchor_kf];
    const Eigen::Matrix4d T = toMatrix(kf->rot, kf->pos) * r.T_anchor_frame;
    writeTumLine(ofs, r.t, T.block<3, 3>(0, 0), T.block<3, 1>(0, 3));
  }
  return true;
}

bool System::saveKeyframesTUM(const std::string& path) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  ofs << std::fixed << std::setprecision(9);
  for (const auto& kf : backend_->keyframes()) writeTumLine(ofs, kf->t, kf->rot, kf->pos);
  return true;
}

bool System::saveFrameDiagCSV(const std::string& path) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  // 协方差只存上三角 21 个元素，列名 cov_ij（i<=j，0-2 旋转，3-5 平移）
  ofs << "t,mode,ok,proc_ms,iterations,num_valid,mean_error,converged,"
         "degenerate_mask,vio_track,td,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z";
  for (int i = 0; i < 6; ++i)
    for (int j = i; j < 6; ++j) ofs << ",cov_" << i << j;
  ofs << "\n" << std::setprecision(9);

  std::lock_guard<std::mutex> lk(traj_mutex_);
  for (const auto& d : frame_diags_) {
    ofs << std::fixed << d.t << std::defaultfloat << "," << d.mode << "," << d.ok << ","
        << d.proc_ms << "," << d.iterations << "," << d.num_valid << ","
        << d.mean_error << "," << d.converged << "," << d.degenerate_mask << ","
        << d.vio_track << "," << d.td << ","
        << d.bg.x() << "," << d.bg.y() << "," << d.bg.z() << ","
        << d.ba.x() << "," << d.ba.y() << "," << d.ba.z();
    for (int i = 0; i < 6; ++i)
      for (int j = i; j < 6; ++j) ofs << "," << d.pose_cov(i, j);
    ofs << "\n";
  }
  return true;
}

bool System::saveLoopEventsCSV(const std::string& path) const {
  std::ofstream ofs(path);
  if (!ofs.is_open()) return false;
  ofs << "stage,id_from,id_to,t_from,t_to,sc_dist,icp_fitness,"
         "dx,dy,dz,qx,qy,qz,qw\n";
  ofs << std::setprecision(9);
  for (const auto& e : backend_->loopEvents()) {
    const char* stage = e.stage == LoopStage::kAccepted       ? "accepted"
                      : e.stage == LoopStage::kRejectedVisual ? "rejected_visual"
                                                              : "rejected_icp";
    const Eigen::Quaterniond q(e.rot_delta);
    ofs << stage << "," << e.id_from << "," << e.id_to << ","
        << std::fixed << e.t_from << "," << e.t_to << std::defaultfloat << ","
        << e.sc_dist << "," << e.icp_fitness << ","
        << e.pos_delta.x() << "," << e.pos_delta.y() << "," << e.pos_delta.z() << ","
        << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";
  }
  return true;
}

bool System::saveMapPCD(const std::string& path, double leaf) const {
  CloudPtr map = buildGlobalMap(leaf);
  if (!map || map->empty()) return false;
  return pcl::io::savePCDFileBinary(path, *map) == 0;
}

}  // namespace lvi
