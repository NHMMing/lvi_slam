// lvi_slam/sync/data_synchronizer.cpp
// 三传感器时间对齐与 LIO/VIO 交替调度。
//
// 整个系统时间正确性的守门员。这里做错了，后面所有数学都白搭。
//
// 核心机制：LIO 和 VIO 在**同一个时刻**依次更新。
//   先把点云和 IMU 积分到图像曝光时刻做一次 LIO 更新，
//   紧接着用同一时刻的图像做 VIO 更新。
// 这样 VIO 的直接法拿到的初值就是刚被 LiDAR 约束过的状态 ——
// 光度误差的收敛域只有几个像素，这个好初值是必需的。
#include "lvi_slam/sync/data_synchronizer.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>

namespace lvi {

DataSynchronizer::DataSynchronizer(const Config& cfg) : cfg_(cfg) {
  cloud_next_.reset(new PointCloud());
}

void DataSynchronizer::pushImu(const ImuData& imu) {
  std::lock_guard<std::mutex> lk(mtx_);
  // 时间戳回跳：数据源重启或 bag 循环播放。清空重来，不要试图修补。
  if (!imu_buf_.empty() && imu.t < imu_buf_.back().t) {
    imu_buf_.clear();
    cloud_buf_.clear();
    cloud_time_buf_.clear();
    image_buf_.clear();
    last_mode_ = UpdateMode::kNone;
    last_update_time_ = -1.0;
  }
  imu_buf_.push_back(imu);
  // IMU 缓冲上限：正常情况下会被及时消费，堆积说明前端跟不上，
  // 这时丢老数据比 OOM 好。
  while (imu_buf_.size() > 20000) { imu_buf_.pop_front(); ++drops_.imu; }
}

void DataSynchronizer::pushCloud(const CloudPtr& cloud, double header_time) {
  if (!cloud || cloud->empty()) return;
  std::lock_guard<std::mutex> lk(mtx_);
  cloud_buf_.push_back(cloud);
  cloud_time_buf_.push_back(header_time);
  while (cloud_buf_.size() > 50) {
    cloud_buf_.pop_front();
    cloud_time_buf_.pop_front();
    ++drops_.cloud;
  }
}

void DataSynchronizer::pushImage(const cv::Mat& img, double timestamp) {
  if (img.empty()) return;
  std::lock_guard<std::mutex> lk(mtx_);
  ImageData d;
  d.t = timestamp;
  d.rgb = img;
  if (img.channels() == 3) cv::cvtColor(img, d.gray, cv::COLOR_BGR2GRAY);
  else d.gray = img;
  image_buf_.push_back(std::move(d));
  while (image_buf_.size() > 30) { image_buf_.pop_front(); ++drops_.image; }
}

void DataSynchronizer::reset() {
  std::lock_guard<std::mutex> lk(mtx_);
  imu_buf_.clear();
  cloud_buf_.clear();
  cloud_time_buf_.clear();
  image_buf_.clear();
  cloud_next_->clear();
  last_mode_ = UpdateMode::kNone;
  last_update_time_ = -1.0;
}

bool DataSynchronizer::collectImu(double from_time, double to_time,
                                  std::vector<ImuData>* out) {
  out->clear();
  if (imu_buf_.empty()) return false;
  if (imu_buf_.back().t < to_time) return false;   // IMU 还没覆盖到，等

  while (!imu_buf_.empty() && imu_buf_.front().t < from_time) imu_buf_.pop_front();
  for (const auto& imu : imu_buf_) {
    if (imu.t > to_time) break;
    out->push_back(imu);
  }
  // 消费掉已使用的部分（保留一个用于下一段的起点衔接）
  while (imu_buf_.size() > 1 && imu_buf_.front().t <= to_time) imu_buf_.pop_front();
  return !out->empty();
}

// ---------------------------------------------------------------------------
// 点云按时间切分。
//
// curvature 字段存「相对帧首的毫秒偏移」（Livox 预处理器填的）。
// 一帧点云可能跨越图像时刻，切开后：
//   前半 → 本轮 LIO 更新
//   后半 → 存入 cloud_next_，下一轮接上
// 切分时重算时间基准，保证 curvature 始终是相对「本轮起始时刻」的偏移。
// ---------------------------------------------------------------------------
void DataSynchronizer::splitCloudByTime(double split_time, CloudPtr cloud_cur) {
  cloud_cur->clear();
  // 先接上上一轮切剩的
  *cloud_cur = *cloud_next_;
  cloud_next_->clear();

  while (!cloud_buf_.empty()) {
    const double frame_t = cloud_time_buf_.front();
    if (frame_t > split_time) break;

    const auto& pts = cloud_buf_.front()->points;
    const float split_ms = static_cast<float>((split_time - frame_t) * 1000.0);

    for (const auto& p : pts) {
      PointType q = p;
      if (p.curvature < split_ms) {
        // 本轮：偏移改为相对本轮起始时刻
        q.curvature += static_cast<float>((frame_t - last_update_time_) * 1000.0);
        cloud_cur->push_back(q);
      } else {
        // 下一轮：偏移改为相对 split_time
        q.curvature += static_cast<float>((frame_t - split_time) * 1000.0);
        cloud_next_->push_back(q);
      }
    }
    cloud_buf_.pop_front();
    cloud_time_buf_.pop_front();
  }
}

bool DataSynchronizer::trySync(SyncPackage* pkg) {
  std::lock_guard<std::mutex> lk(mtx_);
  if (imu_buf_.empty() || cloud_buf_.empty()) return false;

  // ---- 纯 LIO 模式：不等图像 ----
  if (!cfg_.visual.enable) {
    const double frame_t = cloud_time_buf_.front();
    const auto& pts = cloud_buf_.front()->points;
    if (pts.empty()) { cloud_buf_.pop_front(); cloud_time_buf_.pop_front(); return false; }

    const double frame_end = frame_t + pts.back().curvature * 1e-3;
    if (last_update_time_ < 0) last_update_time_ = frame_t;

    pkg->mode = UpdateMode::kLio;
    pkg->sync_time = frame_end;
    pkg->last_update_time = last_update_time_;
    if (!collectImu(last_update_time_, frame_end, &pkg->imus)) return false;

    pkg->cloud = cloud_buf_.front();
    cloud_buf_.pop_front();
    cloud_time_buf_.pop_front();
    last_update_time_ = frame_end;
    last_mode_ = UpdateMode::kLio;
    return true;
  }

  // ---- LIVO 模式：LIO / VIO 交替 ----
  if (image_buf_.empty()) return false;

  // 上一轮是 LIO → 本轮立刻做 VIO，用同一时刻的图像
  if (last_mode_ == UpdateMode::kLio) {
    pkg->mode = UpdateMode::kVio;
    pkg->sync_time = last_update_time_;
    pkg->last_update_time = last_update_time_;
    pkg->imus.clear();          // IMU 已在 LIO 半程消耗，状态已传播到位
    pkg->cloud.reset();
    pkg->image = image_buf_.front();
    image_buf_.pop_front();
    last_mode_ = UpdateMode::kVio;
    return true;
  }

  // 上一轮是 VIO（或初始）→ 本轮做 LIO，积分到下一个图像时刻
  const double img_time = image_buf_.front().t + td_ + exposure_time_;

  // 图像比上次同步还旧 → 图像流落后了，丢弃
  if (last_update_time_ > 0 && img_time <= last_update_time_ + 1e-5) {
    image_buf_.pop_front();
    return false;
  }
  if (last_update_time_ < 0) last_update_time_ = cloud_time_buf_.front();

  // 点云还没覆盖到图像时刻 → 等
  const auto& last_pts = cloud_buf_.back()->points;
  const double cloud_end = cloud_time_buf_.back() +
                           (last_pts.empty() ? 0.0 : last_pts.back().curvature * 1e-3);
  if (cloud_end < img_time) return false;

  if (!collectImu(last_update_time_, img_time, &pkg->imus)) return false;

  CloudPtr cloud_cur(new PointCloud());
  splitCloudByTime(img_time, cloud_cur);
  if (cloud_cur->empty()) return false;

  pkg->mode = UpdateMode::kLio;
  pkg->sync_time = img_time;
  pkg->last_update_time = last_update_time_;
  pkg->cloud = cloud_cur;

  last_update_time_ = img_time;
  last_mode_ = UpdateMode::kLio;
  return true;
}

}  // namespace lvi
