// lvi_slam/io/data_source.h
// 数据源抽象：把「从哪读」和「读到什么」分开。
//
// 关键设计决策：按**时间戳归并**输出，不是按传感器分别遍历。
//
// 离线复现必须严格保持在线时的消息到达顺序，否则 DataSynchronizer 的
// LIO/VIO 交替逻辑走的路径和实际部署时不一样 —— 离线跑得好好的，
// 上车就出问题，而且完全无法复现。
//
// 所以这里输出的是一个按时间排好序的单一消息流，
// offline_runner 只负责按序 feed 给 System，不做任何调度决策。
#pragma once

#include "lvi_slam/common/types.h"
#include "lvi_slam/common/config.h"
#include "lvi_slam/io/cloud_decoder.h"
#include <memory>
#include <string>
#include <functional>

namespace lvi {

enum class MsgType { kImu, kCloud, kImage, kGps, kEnd };

struct SourceMessage {
  MsgType  type = MsgType::kEnd;
  double   t = 0.0;

  ImuData  imu;
  CloudPtr cloud;
  cv::Mat  image;
  V3D      gps_pos = V3D::Zero();
  V3D      gps_cov = V3D::Zero();
};

// 话题映射。不同数据集话题名千奇百怪，全放配置里。
struct TopicMap {
  std::string imu   = "/imu/data";
  std::string lidar = "/points_raw";
  std::string image = "/camera/image_raw";
  std::string gps;                       // 留空表示不读
};

class DataSource {
 public:
  virtual ~DataSource() = default;

  // 取下一条消息（已按时间排序）。返回 false 表示数据读完。
  virtual bool next(SourceMessage* msg) = 0;

  // 用于进度条
  virtual double startTime() const = 0;
  virtual double endTime()   const = 0;
  virtual size_t totalCount() const = 0;

  virtual const char* name() const = 0;

  // 工厂：按路径后缀和目录结构自动选实现
  //   *.bag          → ROS1 bag
  //   *.db3 / *.mcap → ROS2 bag
  //   目录且含 metadata.yaml → ROS2 bag 目录
  //   其他目录       → KITTI 风格文件夹
  static std::unique_ptr<DataSource> create(const std::string& path,
                                            const Config& cfg,
                                            const TopicMap& topics,
                                            const CloudProfile& profile);
};

// ---------------------------------------------------------------------------
// ROS1 bag（.bag）
//
// rosbag::View 跨话题按时间归并，同样不需要额外排序。
// 仅在 ROS1 (catkin) 构建下带真实实现，其余构建里是报错桩。
// ---------------------------------------------------------------------------
class Rosbag1Source : public DataSource {
 public:
  Rosbag1Source(const std::string& path, const Config& cfg,
                const TopicMap& topics, const CloudProfile& profile);
  ~Rosbag1Source() override;

  bool next(SourceMessage* msg) override;
  double startTime() const override { return start_time_; }
  double endTime()   const override { return end_time_; }
  size_t totalCount() const override { return total_count_; }
  const char* name() const override { return "rosbag1"; }

 private:
  struct Impl;                 // pimpl：把 rosbag 的头文件挡在核心库之外
  std::unique_ptr<Impl> impl_;
  double start_time_ = 0, end_time_ = 0;
  size_t total_count_ = 0;
};

// ---------------------------------------------------------------------------
// ROS2 bag（.db3 / .mcap / 目录）
//
// rosbag2_cpp 的 SequentialReader 本身就是按时间顺序返回的，
// 所以这里不需要额外排序 —— 直接透传即可。
// ---------------------------------------------------------------------------
class Ros2BagSource : public DataSource {
 public:
  Ros2BagSource(const std::string& path, const Config& cfg,
                const TopicMap& topics, const CloudProfile& profile);
  ~Ros2BagSource() override;

  bool next(SourceMessage* msg) override;
  double startTime() const override { return start_time_; }
  double endTime()   const override { return end_time_; }
  size_t totalCount() const override { return total_count_; }
  const char* name() const override { return "ros2bag"; }

 private:
  struct Impl;                 // pimpl：把 rosbag2 的头文件挡在核心库之外
  std::unique_ptr<Impl> impl_;
  double start_time_ = 0, end_time_ = 0;
  size_t total_count_ = 0;
};

// ---------------------------------------------------------------------------
// KITTI 风格文件夹
//
// 结构：velodyne_points/data/*.bin + timestamps.txt，oxts/data/*.txt（IMU+GPS）
// 没有逐点时间，必须配合 CloudProfile::kitti() 按方位角反推。
// ---------------------------------------------------------------------------
class KittiFolderSource : public DataSource {
 public:
  KittiFolderSource(const std::string& dir, const Config& cfg,
                    const CloudProfile& profile);

  bool next(SourceMessage* msg) override;
  double startTime() const override { return start_time_; }
  double endTime()   const override { return end_time_; }
  size_t totalCount() const override { return queue_.size(); }
  const char* name() const override { return "kitti_folder"; }

 private:
  // 索引项：只存路径和时间，真正读文件推迟到 next() —— 避免一次性吃满内存
  struct Entry { double t; MsgType type; std::string path; size_t idx; };

  bool scanVelodyne(const std::string& dir);
  bool scanOxts(const std::string& dir);
  bool loadBin(const std::string& path, CloudPtr out) const;

  Config       cfg_;
  CloudProfile profile_;
  std::vector<Entry> queue_;
  std::vector<ImuData> imu_cache_;      // oxts 解析出的 IMU，按索引取
  size_t cursor_ = 0;
  double start_time_ = 0, end_time_ = 0;
};

}  // namespace lvi
