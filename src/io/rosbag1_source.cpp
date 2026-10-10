// lvi_slam/io/rosbag1_source.cpp
// ROS1 bag 读取（.bag）。pimpl 隔离 rosbag 头文件，机制与 ros2bag_source.cpp 一致。
//
// 公开数据集（MARS-LVIG / R3LIVE / FAST-LIVO / Hilti / NTU VIRAL）绝大多数
// 原生就是 ROS1 bag，直接读，不需要先转成 ROS2 格式。
#include "lvi_slam/io/data_source.h"

#ifdef LVI_WITH_ROSBAG1
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <cv_bridge/cv_bridge.h>
#include "lvi_slam/io/livox_msg.h"
#endif

#include <algorithm>
#include <iostream>

namespace lvi {

#ifdef LVI_WITH_ROSBAG1

struct Rosbag1Source::Impl {
  rosbag::Bag bag;
  std::unique_ptr<rosbag::View> view;
  rosbag::View::iterator it;
  TopicMap topics;
  Config   cfg;
  std::unique_ptr<CloudDecoder> decoder;
  bool     lidar_is_livox = false;
};

Rosbag1Source::Rosbag1Source(const std::string& path, const Config& cfg,
                             const TopicMap& topics, const CloudProfile& profile)
    : impl_(std::make_unique<Impl>()) {
  impl_->topics = topics;
  impl_->cfg = cfg;
  impl_->decoder = std::make_unique<CloudDecoder>(cfg, profile);
  impl_->lidar_is_livox = (profile.name == "livox");

  try {
    impl_->bag.open(path, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& e) {
    std::cerr << "[ROS1Bag] 打开失败: " << e.what() << "\n";
    return;
  }

  // 只读需要的话题，理由同 ROS2 版：过滤掉 tf、相机 info、诊断等无关消息。
  std::vector<std::string> wanted = {topics.imu, topics.lidar};
  if (!topics.image.empty()) wanted.push_back(topics.image);
  if (!topics.gps.empty())   wanted.push_back(topics.gps);

  impl_->view = std::make_unique<rosbag::View>(impl_->bag, rosbag::TopicQuery(wanted));
  impl_->it = impl_->view->begin();

  start_time_ = impl_->view->getBeginTime().toSec();
  end_time_   = impl_->view->getEndTime().toSec();
  total_count_ = impl_->view->size();

  std::cout << "[ROS1Bag] " << path << "\n";
  rosbag::View all(impl_->bag);
  for (const auto* ci : all.getConnections()) {
    const bool used = std::find(wanted.begin(), wanted.end(), ci->topic) != wanted.end();
    std::cout << "  " << (used ? "[用] " : "     ") << ci->topic
              << "  (" << ci->datatype << ")\n";
  }
  if (total_count_ == 0) {
    std::cerr << "[ROS1Bag] 警告：过滤后一条消息都没有，检查话题名配置\n";
  }
}

Rosbag1Source::~Rosbag1Source() = default;

bool Rosbag1Source::next(SourceMessage* msg) {
  if (!impl_->view) { msg->type = MsgType::kEnd; return false; }

  // rosbag::View 按 bag 时间跨话题归并，顺序与录制时一致。
  // 时间戳仍然用消息自带的 header.stamp，不用 bag 接收时间（理由同 ROS2 版）。
  while (impl_->it != impl_->view->end()) {
    const rosbag::MessageInstance m = *impl_->it;
    ++impl_->it;
    const std::string& topic = m.getTopic();

    if (topic == impl_->topics.imu) {
      auto p = m.instantiate<sensor_msgs::Imu>();
      if (!p) continue;
      msg->type = MsgType::kImu;
      msg->t = p->header.stamp.toSec();
      msg->imu.t = msg->t;
      msg->imu.acc = V3D(p->linear_acceleration.x, p->linear_acceleration.y,
                         p->linear_acceleration.z);
      msg->imu.gyr = V3D(p->angular_velocity.x, p->angular_velocity.y,
                         p->angular_velocity.z);
      return true;
    }

    if (topic == impl_->topics.lidar) {
      msg->cloud.reset(new PointCloud());

      if (impl_->lidar_is_livox) {
        auto p = m.instantiate<LivoxCustomMsg>();
        if (!p) continue;
        msg->t = p->header.stamp.toSec();

        const int fnum = std::max(1, impl_->cfg.lidar.point_filter_num);
        const double bmin2 = impl_->cfg.lidar.blind_min * impl_->cfg.lidar.blind_min;
        const double bmax2 = impl_->cfg.lidar.blind_max * impl_->cfg.lidar.blind_max;

        for (uint32_t i = 0; i < p->point_num; ++i) {
          if (static_cast<int>(i) % fnum != 0) continue;
          const auto& pt = p->points[i];
          if (impl_->cfg.lidar.use_livox_tag) {
            if ((pt.tag & 0x03) != 0 || ((pt.tag >> 2) & 0x03) != 0) continue;
          }
          const double d2 = static_cast<double>(pt.x) * pt.x +
                            static_cast<double>(pt.y) * pt.y +
                            static_cast<double>(pt.z) * pt.z;
          if (d2 < bmin2 || d2 > bmax2) continue;

          PointType q;
          q.x = pt.x; q.y = pt.y; q.z = pt.z;
          q.intensity = pt.reflectivity;
          q.curvature = static_cast<float>(pt.offset_time) * 1e-6f;   // ns → ms
          msg->cloud->push_back(q);
        }
      } else {
        auto p = m.instantiate<sensor_msgs::PointCloud2>();
        if (!p) continue;
        msg->t = p->header.stamp.toSec();

        RawCloud2 raw;
        raw.header_time = msg->t;
        raw.width = p->width;  raw.height = p->height;
        raw.point_step = p->point_step; raw.row_step = p->row_step;
        raw.is_dense = p->is_dense;
        for (const auto& f : p->fields) {
          raw.fields.push_back({f.name, f.offset, f.datatype, f.count});
        }
        raw.data = p->data.data();
        raw.data_size = p->data.size();
        impl_->decoder->decode(raw, msg->cloud);
      }

      if (msg->cloud->empty()) continue;   // 整帧被滤空，跳过
      msg->type = MsgType::kCloud;
      return true;
    }

    if (!impl_->topics.image.empty() && topic == impl_->topics.image) {
      auto p = m.instantiate<sensor_msgs::Image>();
      if (!p) continue;
      msg->type = MsgType::kImage;
      msg->t = p->header.stamp.toSec();
      try {
        auto ptr = cv_bridge::toCvCopy(*p, p->encoding == "mono8" ? "mono8" : "bgr8");
        msg->image = ptr->image;
      } catch (const std::exception& e) {
        std::cerr << "[ROS1Bag] 图像解码失败: " << e.what() << "\n";
        continue;
      }
      return true;
    }

    if (!impl_->topics.gps.empty() && topic == impl_->topics.gps) {
      auto p = m.instantiate<sensor_msgs::NavSatFix>();
      if (!p) continue;
      msg->type = MsgType::kGps;
      msg->t = p->header.stamp.toSec();
      msg->gps_pos = V3D(p->latitude, p->longitude, p->altitude);
      msg->gps_cov = V3D(p->position_covariance[0], p->position_covariance[4],
                         p->position_covariance[8]);
      return true;
    }
  }
  msg->type = MsgType::kEnd;
  return false;
}

#else   // 编译时未启用 rosbag1 的桩

struct Rosbag1Source::Impl {};

Rosbag1Source::Rosbag1Source(const std::string&, const Config&,
                             const TopicMap&, const CloudProfile&) {
  std::cerr << "[ROS1Bag] 本二进制编译时未启用 ROS1 bag 支持。\n"
            << "  在 ROS1 (catkin) 环境重新编译，或改用 ROS2 bag / 文件夹数据源。\n";
}
Rosbag1Source::~Rosbag1Source() = default;
bool Rosbag1Source::next(SourceMessage* msg) { msg->type = MsgType::kEnd; return false; }

#endif

}  // namespace lvi
