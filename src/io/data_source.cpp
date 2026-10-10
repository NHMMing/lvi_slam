// lvi_slam/io/data_source.cpp
#include "lvi_slam/io/data_source.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <algorithm>

namespace fs = std::filesystem;

namespace lvi {

std::unique_ptr<DataSource> DataSource::create(const std::string& path,
                                               const Config& cfg,
                                               const TopicMap& topics,
                                               const CloudProfile& profile) {
  if (!fs::exists(path)) {
    std::cerr << "[DataSource] 路径不存在: " << path << "\n";
    return nullptr;
  }

  if (fs::is_directory(path)) {
    // ROS2 bag 目录的标志是 metadata.yaml
    if (fs::exists(fs::path(path) / "metadata.yaml")) {
      return std::make_unique<Ros2BagSource>(path, cfg, topics, profile);
    }
    // 否则当 KITTI 风格文件夹
    return std::make_unique<KittiFolderSource>(path, cfg, profile);
  }

  const std::string ext = fs::path(path).extension().string();
  if (ext == ".db3" || ext == ".mcap") {
    return std::make_unique<Ros2BagSource>(path, cfg, topics, profile);
  }
  if (ext == ".bag") {
    // ROS1 bag。MARS-LVIG / R3LIVE / Hilti / NTU VIRAL 都是这个格式。
    // ROS1 (catkin) 构建下直接读；其他构建里 Rosbag1Source 是报错桩。
    return std::make_unique<Rosbag1Source>(path, cfg, topics, profile);
  }

  std::cerr << "[DataSource] 无法识别的格式: " << ext << "\n";
  return nullptr;
}

// ---------------------------------------------------------------------------
// KittiFolderSource
// ---------------------------------------------------------------------------
KittiFolderSource::KittiFolderSource(const std::string& dir, const Config& cfg,
                                     const CloudProfile& profile)
    : cfg_(cfg), profile_(profile) {
  if (!scanVelodyne(dir)) {
    std::cerr << "[KITTI] 未找到 velodyne_points/data\n";
    return;
  }
  scanOxts(dir);   // IMU 可选：没有的话只能跑纯 LiDAR 里程计

  // 按时间归并排序 —— 这一步是离线复现在线行为的关键
  std::sort(queue_.begin(), queue_.end(),
            [](const Entry& a, const Entry& b) { return a.t < b.t; });

  if (!queue_.empty()) {
    start_time_ = queue_.front().t;
    end_time_   = queue_.back().t;
  }
  std::cout << "[KITTI] 索引完成: " << queue_.size() << " 条消息, "
            << (end_time_ - start_time_) << " 秒\n";
}

bool KittiFolderSource::scanVelodyne(const std::string& dir) {
  const fs::path data_dir = fs::path(dir) / "velodyne_points" / "data";
  const fs::path ts_file  = fs::path(dir) / "velodyne_points" / "timestamps.txt";
  if (!fs::exists(data_dir)) return false;

  std::vector<std::string> files;
  for (const auto& e : fs::directory_iterator(data_dir)) {
    if (e.path().extension() == ".bin") files.push_back(e.path().string());
  }
  std::sort(files.begin(), files.end());

  // timestamps.txt 每行一个 "YYYY-MM-DD HH:MM:SS.nnnnnnnnn"
  std::vector<double> stamps;
  if (std::ifstream ifs(ts_file); ifs.is_open()) {
    std::string line;
    while (std::getline(ifs, line)) {
      if (line.size() < 19) continue;
      std::tm tm{};
      double frac = 0;
      if (std::sscanf(line.c_str(), "%d-%d-%d %d:%d:%lf",
                      &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
                      &tm.tm_hour, &tm.tm_min, &frac) == 6) {
        tm.tm_year -= 1900; tm.tm_mon -= 1;
        tm.tm_sec = static_cast<int>(frac);
        stamps.push_back(static_cast<double>(timegm(&tm)) + (frac - tm.tm_sec));
      }
    }
  }

  for (size_t i = 0; i < files.size(); ++i) {
    // 没有 timestamps.txt 时按 10Hz 伪造 —— 只能用于粗略验证，不能用于精度评估
    const double t = (i < stamps.size()) ? stamps[i] : i * 0.1;
    queue_.push_back({t, MsgType::kCloud, files[i], i});
  }
  if (stamps.empty()) {
    std::cerr << "[KITTI] 警告：无 timestamps.txt，按 10Hz 伪造时间戳。\n"
              << "        这会让 IMU 对齐失效，仅供冒烟测试。\n";
  }
  return !queue_.empty();
}

bool KittiFolderSource::scanOxts(const std::string& dir) {
  const fs::path data_dir = fs::path(dir) / "oxts" / "data";
  if (!fs::exists(data_dir)) return false;

  std::vector<std::string> files;
  for (const auto& e : fs::directory_iterator(data_dir)) {
    if (e.path().extension() == ".txt") files.push_back(e.path().string());
  }
  std::sort(files.begin(), files.end());

  // oxts 每行 30 个字段，我们只要加速度和角速度。
  // 索引依据 KITTI 官方 dataformat.txt：
  //   [11..13] ax ay az (车体系), [17..19] wx wy wz (车体系)
  for (size_t i = 0; i < files.size(); ++i) {
    std::ifstream ifs(files[i]);
    if (!ifs.is_open()) continue;
    std::vector<double> v;
    double x;
    while (ifs >> x) v.push_back(x);
    if (v.size() < 20) continue;

    ImuData imu;
    imu.acc = V3D(v[11], v[12], v[13]);
    imu.gyr = V3D(v[17], v[18], v[19]);
    imu_cache_.push_back(imu);
  }

  // KITTI 的 oxts 是 100Hz，但只按帧给一个文件。
  // 这意味着 IMU 频率实际上等于图像帧率（10Hz）—— 对紧耦合系统远远不够。
  // 这是 KITTI 不适合评测 LIO/LVIO 的根本原因，只能用来测纯激光里程计。
  std::cerr << "[KITTI] 警告：oxts 只有 " << imu_cache_.size()
            << " 条 IMU（约 10Hz）。\n"
            << "        紧耦合需要 100Hz+，KITTI 不适合评测 LIO/LVIO，\n"
            << "        建议改用 MARS-LVIG / NTU VIRAL / Hilti。\n";
  return true;
}

bool KittiFolderSource::loadBin(const std::string& path, CloudPtr out) const {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) return false;
  ifs.seekg(0, std::ios::end);
  const size_t bytes = ifs.tellg();
  ifs.seekg(0, std::ios::beg);

  const size_t n = bytes / (4 * sizeof(float));   // x y z intensity
  std::vector<float> buf(n * 4);
  ifs.read(reinterpret_cast<char*>(buf.data()), bytes);

  out->clear();
  out->reserve(n / std::max(1, cfg_.lidar.point_filter_num));

  const double bmin2 = cfg_.lidar.blind_min * cfg_.lidar.blind_min;
  const double bmax2 = cfg_.lidar.blind_max * cfg_.lidar.blind_max;

  for (size_t i = 0; i < n; ++i) {
    if (i % cfg_.lidar.point_filter_num != 0) continue;
    const float x = buf[i * 4], y = buf[i * 4 + 1], z = buf[i * 4 + 2];
    const double d2 = static_cast<double>(x) * x + static_cast<double>(y) * y +
                      static_cast<double>(z) * z;
    if (d2 < bmin2 || d2 > bmax2) continue;

    PointType p;
    p.x = x; p.y = y; p.z = z;
    p.intensity = buf[i * 4 + 3];

    // 按方位角反推逐点时间（KITTI 无时间字段）
    double az = std::atan2(y, x);
    if (az < 0) az += 2.0 * M_PI;
    const double ratio = profile_.clockwise ? (1.0 - az / (2.0 * M_PI))
                                            : (az / (2.0 * M_PI));
    p.curvature = static_cast<float>(ratio * profile_.scan_period * 1000.0);
    out->push_back(p);
  }
  return !out->empty();
}

bool KittiFolderSource::next(SourceMessage* msg) {
  if (cursor_ >= queue_.size()) { msg->type = MsgType::kEnd; return false; }
  const Entry& e = queue_[cursor_++];

  msg->t = e.t;
  msg->type = e.type;
  if (e.type == MsgType::kCloud) {
    msg->cloud.reset(new PointCloud());
    if (!loadBin(e.path, msg->cloud)) return next(msg);   // 坏文件跳过
  } else if (e.type == MsgType::kImu) {
    if (e.idx < imu_cache_.size()) {
      msg->imu = imu_cache_[e.idx];
      msg->imu.t = e.t;
    }
  }
  return true;
}

}  // namespace lvi
