// lvi_slam/sync/data_synchronizer.h
// 多传感器数据同步与切分。
//
// 这一层决定了 LIO 和 VIO 更新的时刻对齐质量，是 LVI 系统精度的隐形基础。
//
// 借鉴 FAST-LIVO2：点云按图像曝光时刻精确切分成两半，保证 LIO 更新和
// 紧随其后的 VIO 更新作用在同一时刻的状态上。
//
// 借鉴 VINS-Mono：时间偏移 td 作为状态量在线估计。
//   FAST-LIVO2 假设硬件同步——这在实验室成立，在消费级硬件（软件时间戳）
//   上不成立，几十毫秒的偏移很常见。加 td 的代价只是状态多 1 维，
//   收益是系统能在没有硬件同步的平台上工作。
#pragma once

#include "lvi_slam/common/types.h"
#include "lvi_slam/common/config.h"
#include <deque>
#include <mutex>

namespace lvi {

class DataSynchronizer {
 public:
  explicit DataSynchronizer(const Config& cfg);

  // 传感器回调（线程安全）
  void pushImu(const ImuData& imu);
  void pushCloud(const CloudPtr& cloud, double header_time);
  void pushImage(const cv::Mat& img, double timestamp);

  // 取出一个可处理的观测包。
  //
  // 状态机在 LIO 和 VIO 之间交替：
  //   上一轮是 VIO（或初始） -> 本轮 LIO：把点云和 IMU 积分到图像曝光时刻
  //   上一轮是 LIO           -> 本轮 VIO：用同一时刻的图像做光度更新
  //
  // 返回 false 表示数据还不够，调用方应等待。
  bool trySync(SyncPackage* pkg);

  // td 由 iEKF 在线估计，同步器需要拿到最新值来算图像的实际曝光时刻
  void setTimeOffset(double td) { td_ = td; }

  void reset();

  // 尚未被 trySync 取走的点云帧数。离线回放用它做背压（见 offline_runner）。
  size_t pendingClouds() {
    std::lock_guard<std::mutex> lk(mtx_);
    return cloud_buf_.size();
  }

 private:
  // 把点云按 split_time 切成两半：
  //   之前的进 cloud_cur（本轮 LIO 处理）
  //   之后的存 cloud_next（留给下一轮）
  // 点的时间偏移存在 PointType 的 curvature 字段（毫秒，相对帧首）
  void splitCloudByTime(double split_time, CloudPtr cloud_cur);

  bool collectImu(double from_time, double to_time,
                  std::vector<ImuData>* out);

  Config cfg_;
  std::mutex mtx_;

  std::deque<ImuData>  imu_buf_;
  std::deque<CloudPtr> cloud_buf_;
  std::deque<double>   cloud_time_buf_;
  std::deque<ImageData> image_buf_;

  // 双缓冲：跨越图像时刻的点云被切开，后半部分暂存
  CloudPtr cloud_next_;

  UpdateMode last_mode_ = UpdateMode::kNone;
  double last_update_time_ = -1.0;
  double td_ = 0.0;
  double exposure_time_ = 0.0;   // 曝光时间常量偏移（配置项）
};

}  // namespace lvi
