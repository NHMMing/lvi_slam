// ros2/lvi_slam_node.cpp
// ROS2 包装层。只做三件事：消息转换、参数加载、结果发布。
// 一行算法逻辑都不放在这里 —— 核心库完全不依赖 ROS。
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "lvi_slam/system.h"
#include "lvi_slam/preprocess/livox_preprocess.h"

namespace {

// ROS 时间 → 秒。全系统统一用 double 秒，避免 rclcpp::Time 到处传。
inline double toSec(const builtin_interfaces::msg::Time& t) {
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

}  // namespace

class LviSlamNode : public rclcpp::Node {
 public:
  LviSlamNode() : Node("lvi_slam") {
    const std::string cfg_path =
        declare_parameter<std::string>("config_file", "config/livox_mid360.yaml");

    if (!cfg_.loadFromFile(cfg_path)) {
      RCLCPP_FATAL(get_logger(), "配置加载失败: %s", cfg_path.c_str());
      throw std::runtime_error("config load failed");
    }
    RCLCPP_INFO(get_logger(), "配置已加载: %s", cfg_path.c_str());

    system_ = std::make_unique<lvi::System>(cfg_);
    preprocess_ = std::make_unique<lvi::LivoxPreprocess>(cfg_);

    // IMU 用 SensorDataQoS（BestEffort + 深队列）。
    // IMU 200Hz+，用默认 Reliable QoS 在丢包时会重传，反而增加延迟。
    // 状态估计宁可丢一帧也不要等一帧。
    auto imu_qos = rclcpp::SensorDataQoS().keep_last(2000);
    auto lidar_qos = rclcpp::SensorDataQoS().keep_last(10);
    auto img_qos = rclcpp::SensorDataQoS().keep_last(10);

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        cfg_.imu_topic, imu_qos,
        std::bind(&LviSlamNode::imuCallback, this, std::placeholders::_1));

    sub_livox_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        cfg_.lidar_topic, lidar_qos,
        std::bind(&LviSlamNode::livoxCallback, this, std::placeholders::_1));

    if (cfg_.visual.enable) {
      sub_img_ = create_subscription<sensor_msgs::msg::Image>(
          cfg_.image_topic, img_qos,
          std::bind(&LviSlamNode::imageCallback, this, std::placeholders::_1));
    }

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("~/odometry", 100);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
    pub_map_  = create_publisher<sensor_msgs::msg::PointCloud2>("~/global_map",
                                 rclcpp::QoS(1).transient_local());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    path_.header.frame_id = "map";

    // 前端每完成一次更新就回调，发布高频位姿
    system_->setOdometryCallback(
        [this](const lvi::State& s, double t) { publishOdometry(s, t); });

    system_->start();

    // 全局地图低频发布 —— 拼接整张图很贵，2 秒一次足够可视化用
    map_timer_ = create_wall_timer(std::chrono::seconds(2),
                                   std::bind(&LviSlamNode::publishMap, this));

    RCLCPP_INFO(get_logger(), "LVI-SLAM 已启动 | LiDAR=%s IMU=%s 视觉=%s",
                cfg_.lidar_topic.c_str(), cfg_.imu_topic.c_str(),
                cfg_.visual.enable ? "开" : "关");
  }

  ~LviSlamNode() override {
    if (system_) system_->stop();
  }

 private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    lvi::ImuData imu;
    imu.t = toSec(msg->header.stamp);
    imu.acc = lvi::V3D(msg->linear_acceleration.x, msg->linear_acceleration.y,
                       msg->linear_acceleration.z);
    imu.gyr = lvi::V3D(msg->angular_velocity.x, msg->angular_velocity.y,
                       msg->angular_velocity.z);
    system_->feedImu(imu);
  }

  // Livox CustomMsg → 内部点云。
  //
  // 关键是时间戳：CustomMsg 的 timebase 是帧首的绝对时间（纳秒），
  // 每个点的 offset_time 是相对帧首的纳秒偏移。这个精度比机械式雷达高得多，
  // 去畸变能做得更准 —— 不用像 Velodyne 那样靠 ring 和角度反推时间。
  void livoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    raw_buf_.clear();
    raw_buf_.reserve(msg->point_num);
    for (uint32_t i = 0; i < msg->point_num; ++i) {
      const auto& p = msg->points[i];
      lvi::LivoxRawPoint q;
      q.x = p.x; q.y = p.y; q.z = p.z;
      q.reflectivity = static_cast<float>(p.reflectivity);
      q.tag = p.tag;
      q.line = p.line;
      q.offset_time = p.offset_time;
      raw_buf_.push_back(q);
    }

    const double header_time = toSec(msg->header.stamp);
    lvi::CloudPtr cloud(new lvi::PointCloud());
    preprocess_->process(raw_buf_, header_time, cloud);

    if (cloud->empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "本帧点云过滤后为空，检查 blind/tag 参数");
      return;
    }
    system_->feedCloud(cloud, header_time);
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat img;
    try {
      img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败: %s", e.what());
      return;
    }
    system_->feedImage(img, toSec(msg->header.stamp));
  }

  void publishOdometry(const lvi::State& s, double t) {
    const Eigen::Quaterniond q(s.rot);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
    odom.header.frame_id = "map";
    odom.child_frame_id = "body";
    odom.pose.pose.position.x = s.pos.x();
    odom.pose.pose.position.y = s.pos.y();
    odom.pose.pose.position.z = s.pos.z();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.twist.twist.linear.x = s.vel.x();
    odom.twist.twist.linear.y = s.vel.y();
    odom.twist.twist.linear.z = s.vel.z();

    // 协方差按 ROS 约定填成 6x6 行主序。
    // 下游（比如 robot_localization 或导航栈）真的会用这个，别填 0。
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        odom.pose.covariance[i * 6 + j] = s.cov(i, j);

    pub_odom_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = "body";
    tf.transform.translation.x = s.pos.x();
    tf.transform.translation.y = s.pos.y();
    tf.transform.translation.z = s.pos.z();
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    // 路径只做可视化，降频存储防止 RViz 卡死
    if (++path_counter_ % 10 == 0) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = odom.header;
      ps.pose = odom.pose.pose;
      path_.poses.push_back(ps);
      path_.header.stamp = odom.header.stamp;
      pub_path_->publish(path_);
    }
  }

  void publishMap() {
    if (pub_map_->get_subscription_count() == 0) return;   // 没人订阅就别拼了
    lvi::CloudPtr map = system_->buildGlobalMap(0.2);
    if (!map || map->empty()) return;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    pub_map_->publish(msg);
  }

  lvi::Config cfg_;
  std::unique_ptr<lvi::System> system_;
  std::unique_ptr<lvi::LivoxPreprocess> preprocess_;
  std::vector<lvi::LivoxRawPoint> raw_buf_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_livox_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr map_timer_;

  nav_msgs::msg::Path path_;
  uint64_t path_counter_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 多线程执行器：IMU 回调频率高，不能被点云/图像回调阻塞。
  // 单线程 executor 下，一次 20ms 的点云处理会让 4 个 IMU 消息排队。
  auto node = std::make_shared<LviSlamNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
