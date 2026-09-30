// lvi_slam/io/ros2bag_source.cpp
// ROS2 bag 读取。用 pimpl 把 rosbag2 的头文件挡在核心库接口之外。
//
// 为什么这一层单独成文件：rosbag2_cpp 依赖整套 ROS2 运行时。
// 如果把它写进 data_source.cpp，整个核心库就没法在无 ROS 环境编译了。
// 这里用条件编译隔离：有 ROS2 就编，没有就给一个报错桩。
#include "lvi_slam/io/data_source.h"

#ifdef LVI_WITH_ROSBAG2
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <cv_bridge/cv_bridge.h>
#endif

#include <iostream>

namespace lvi {

#ifdef LVI_WITH_ROSBAG2

struct Ros2BagSource::Impl {
  rosbag2_cpp::Reader reader;
  TopicMap topics;
  Config   cfg;
  std::unique_ptr<CloudDecoder> decoder;
  bool     lidar_is_livox = false;

  rclcpp::Serialization<sensor_msgs::msg::Imu>          imu_ser;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2>  pc2_ser;
  rclcpp::Serialization<sensor_msgs::msg::Image>        img_ser;
  rclcpp::Serialization<sensor_msgs::msg::NavSatFix>    gps_ser;
  rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> livox_ser;
};

Ros2BagSource::Ros2BagSource(const std::string& path, const Config& cfg,
                             const TopicMap& topics, const CloudProfile& profile)
    : impl_(std::make_unique<Impl>()) {
  impl_->topics = topics;
  impl_->cfg = cfg;
  impl_->decoder = std::make_unique<CloudDecoder>(cfg, profile);
  impl_->lidar_is_livox = (profile.name == "livox");

  rosbag2_storage::StorageOptions so;
  so.uri = path;
  impl_->reader.open(so);

  // 只读需要的话题。bag 里往往还有相机 info、tf、诊断信息等一堆无关数据，
  // 不过滤的话反序列化开销能翻好几倍。
  rosbag2_storage::StorageFilter filter;
  filter.topics = {topics.imu, topics.lidar};
  if (!topics.image.empty()) filter.topics.push_back(topics.image);
  if (!topics.gps.empty())   filter.topics.push_back(topics.gps);
  impl_->reader.set_filter(filter);

  const auto meta = impl_->reader.get_metadata();
  start_time_ = std::chrono::duration<double>(
      meta.starting_time.time_since_epoch()).count();
  end_time_ = start_time_ + std::chrono::duration<double>(meta.duration).count();

  total_count_ = 0;
  std::cout << "[ROS2Bag] " << path << "\n";
  for (const auto& t : meta.topics_with_message_count) {
    const auto& name = t.topic_metadata.name;
    const bool used = std::find(filter.topics.begin(), filter.topics.end(), name)
                      != filter.topics.end();
    std::cout << "  " << (used ? "[用] " : "     ") << name
              << "  (" << t.topic_metadata.type << ")  "
              << t.message_count << " 条\n";
    if (used) total_count_ += t.message_count;
  }
  if (total_count_ == 0) {
    std::cerr << "[ROS2Bag] 警告：过滤后一条消息都没有，检查话题名配置\n";
  }
}

Ros2BagSource::~Ros2BagSource() = default;

bool Ros2BagSource::next(SourceMessage* msg) {
  while (impl_->reader.has_next()) {
    auto bag_msg = impl_->reader.read_next();
    const std::string& topic = bag_msg->topic_name;
    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);

    // 注意：用消息自带的 header.stamp，不是 bag 的接收时间戳。
    // 两者可能差几十毫秒（驱动到 bag 的传输延迟），
    // 用错会让所有传感器的相对时序都偏，去畸变和 LIO/VIO 对齐全乱。
    if (topic == impl_->topics.imu) {
      sensor_msgs::msg::Imu m;
      impl_->imu_ser.deserialize_message(&serialized, &m);
      msg->type = MsgType::kImu;
      msg->t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
      msg->imu.t = msg->t;
      msg->imu.acc = V3D(m.linear_acceleration.x, m.linear_acceleration.y,
                         m.linear_acceleration.z);
      msg->imu.gyr = V3D(m.angular_velocity.x, m.angular_velocity.y,
                         m.angular_velocity.z);
      return true;
    }

    if (topic == impl_->topics.lidar) {
      msg->cloud.reset(new PointCloud());

      if (impl_->lidar_is_livox) {
        // Livox CustomMsg：自定义结构，不走 PointCloud2 解码路径
        livox_ros_driver2::msg::CustomMsg m;
        impl_->livox_ser.deserialize_message(&serialized, &m);
        msg->t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;

        const int fnum = std::max(1, impl_->cfg.lidar.point_filter_num);
        const double bmin2 = impl_->cfg.lidar.blind_min * impl_->cfg.lidar.blind_min;
        const double bmax2 = impl_->cfg.lidar.blind_max * impl_->cfg.lidar.blind_max;

        for (uint32_t i = 0; i < m.point_num; ++i) {
          if (static_cast<int>(i) % fnum != 0) continue;
          const auto& p = m.points[i];
          if (impl_->cfg.lidar.use_livox_tag) {
            if ((p.tag & 0x03) != 0 || ((p.tag >> 2) & 0x03) != 0) continue;
          }
          const double d2 = static_cast<double>(p.x) * p.x +
                            static_cast<double>(p.y) * p.y +
                            static_cast<double>(p.z) * p.z;
          if (d2 < bmin2 || d2 > bmax2) continue;

          PointType q;
          q.x = p.x; q.y = p.y; q.z = p.z;
          q.intensity = p.reflectivity;
          q.curvature = static_cast<float>(p.offset_time) * 1e-6f;   // ns → ms
          msg->cloud->push_back(q);
        }
      } else {
        sensor_msgs::msg::PointCloud2 m;
        impl_->pc2_ser.deserialize_message(&serialized, &m);
        msg->t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;

        RawCloud2 raw;
        raw.header_time = msg->t;
        raw.width = m.width;  raw.height = m.height;
        raw.point_step = m.point_step; raw.row_step = m.row_step;
        raw.is_dense = m.is_dense;
        for (const auto& f : m.fields) {
          raw.fields.push_back({f.name, f.offset, f.datatype, f.count});
        }
        raw.data = m.data.data();
        raw.data_size = m.data.size();
        impl_->decoder->decode(raw, msg->cloud);
      }

      if (msg->cloud->empty()) continue;   // 整帧被滤空，跳过
      msg->type = MsgType::kCloud;
      return true;
    }

    if (!impl_->topics.image.empty() && topic == impl_->topics.image) {
      sensor_msgs::msg::Image m;
      impl_->img_ser.deserialize_message(&serialized, &m);
      msg->type = MsgType::kImage;
      msg->t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
      try {
        auto ptr = cv_bridge::toCvCopy(m, m.encoding == "mono8" ? "mono8" : "bgr8");
        msg->image = ptr->image;
      } catch (const std::exception& e) {
        std::cerr << "[ROS2Bag] 图像解码失败: " << e.what() << "\n";
        continue;
      }
      return true;
    }

    if (!impl_->topics.gps.empty() && topic == impl_->topics.gps) {
      sensor_msgs::msg::NavSatFix m;
      impl_->gps_ser.deserialize_message(&serialized, &m);
      // 经纬度 → 局部 ENU 需要一个原点。这里只透传原始值，
      // 转换逻辑放在 System::feedGps 里（它知道第一帧在哪）。
      msg->type = MsgType::kGps;
      msg->t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
      msg->gps_pos = V3D(m.latitude, m.longitude, m.altitude);
      msg->gps_cov = V3D(m.position_covariance[0], m.position_covariance[4],
                         m.position_covariance[8]);
      return true;
    }
  }
  msg->type = MsgType::kEnd;
  return false;
}

#else   // 无 ROS2 环境的编译桩

struct Ros2BagSource::Impl {};

Ros2BagSource::Ros2BagSource(const std::string&, const Config&,
                             const TopicMap&, const CloudProfile&) {
  std::cerr << "[ROS2Bag] 本二进制编译时未启用 rosbag2 支持。\n"
            << "  重新编译时加 -DLVI_WITH_ROSBAG2=ON，或改用文件夹数据源。\n";
}
Ros2BagSource::~Ros2BagSource() = default;
bool Ros2BagSource::next(SourceMessage* msg) { msg->type = MsgType::kEnd; return false; }

#endif

}  // namespace lvi
