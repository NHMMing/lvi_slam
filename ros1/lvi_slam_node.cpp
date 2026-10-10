// ros1/lvi_slam_node.cpp
// ROS1 (Noetic) 包装层，与 ros2/lvi_slam_node.cpp 职责完全一致：
// 消息转换、参数加载、结果发布。核心库不依赖 ROS，算法逻辑一行都不放在这里。
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lvi_slam/io/livox_msg.h"
#include "lvi_slam/system.h"
#include "lvi_slam/preprocess/livox_preprocess.h"

class LviSlamNode {
 public:
  LviSlamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    std::string cfg_path;
    pnh.param<std::string>("config_file", cfg_path, "config/livox_mid360.yaml");

    if (!cfg_.loadFromFile(cfg_path)) {
      ROS_FATAL("配置加载失败: %s", cfg_path.c_str());
      throw std::runtime_error("config load failed");
    }
    ROS_INFO("配置已加载: %s", cfg_path.c_str());

    system_ = std::make_unique<lvi::System>(cfg_);
    preprocess_ = std::make_unique<lvi::LivoxPreprocess>(cfg_);

    // 队列深度对应 ROS2 版的 SensorDataQoS：IMU 要深，宁可丢帧也不要让回调排队。
    // tcpNoDelay 避免 Nagle 算法给高频 IMU 增加延迟。
    sub_imu_ = nh.subscribe(cfg_.imu_topic, 2000, &LviSlamNode::imuCallback, this,
                            ros::TransportHints().tcpNoDelay());
    sub_livox_ = nh.subscribe(cfg_.lidar_topic, 10, &LviSlamNode::livoxCallback, this,
                              ros::TransportHints().tcpNoDelay());
    if (cfg_.visual.enable) {
      sub_img_ = nh.subscribe(cfg_.image_topic, 10, &LviSlamNode::imageCallback, this);
    }

    pub_odom_ = pnh.advertise<nav_msgs::Odometry>("odometry", 100);
    pub_path_ = pnh.advertise<nav_msgs::Path>("path", 10);
    pub_map_  = pnh.advertise<sensor_msgs::PointCloud2>("global_map", 1, /*latch=*/true);

    path_.header.frame_id = "map";

    system_->setOdometryCallback(
        [this](const lvi::State& s, double t) { publishOdometry(s, t); });
    system_->start();

    // 全局地图低频发布 —— 拼接整张图很贵，2 秒一次足够可视化用
    map_timer_ = nh.createWallTimer(ros::WallDuration(2.0),
                                    &LviSlamNode::publishMap, this);

    ROS_INFO("LVI-SLAM 已启动 | LiDAR=%s IMU=%s 视觉=%s",
             cfg_.lidar_topic.c_str(), cfg_.imu_topic.c_str(),
             cfg_.visual.enable ? "开" : "关");
  }

  ~LviSlamNode() {
    if (system_) system_->stop();
  }

 private:
  void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
    lvi::ImuData imu;
    imu.t = msg->header.stamp.toSec();
    imu.acc = lvi::V3D(msg->linear_acceleration.x, msg->linear_acceleration.y,
                       msg->linear_acceleration.z);
    imu.gyr = lvi::V3D(msg->angular_velocity.x, msg->angular_velocity.y,
                       msg->angular_velocity.z);
    system_->feedImu(imu);
  }

  // Livox CustomMsg → 内部点云。时间戳语义同 ROS2 版：
  // header.stamp 是帧首绝对时间，每点 offset_time 是相对帧首的纳秒偏移。
  void livoxCallback(const lvi::LivoxCustomMsg::ConstPtr& msg) {
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

    const double header_time = msg->header.stamp.toSec();
    lvi::CloudPtr cloud(new lvi::PointCloud());
    preprocess_->process(raw_buf_, header_time, cloud);

    if (cloud->empty()) {
      ROS_WARN_THROTTLE(5.0, "本帧点云过滤后为空，检查 blind/tag 参数");
      return;
    }
    system_->feedCloud(cloud, header_time);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat img;
    try {
      img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge 转换失败: %s", e.what());
      return;
    }
    system_->feedImage(img, msg->header.stamp.toSec());
  }

  void publishOdometry(const lvi::State& s, double t) {
    const Eigen::Quaterniond q(s.rot);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time().fromSec(t);
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

    // 协方差按 ROS 约定填成 6x6 行主序，下游真的会用，别填 0。
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        odom.pose.covariance[i * 6 + j] = s.cov(i, j);

    pub_odom_.publish(odom);

    geometry_msgs::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = "body";
    tf.transform.translation.x = s.pos.x();
    tf.transform.translation.y = s.pos.y();
    tf.transform.translation.z = s.pos.z();
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_.sendTransform(tf);

    // 路径只做可视化，降频存储防止 RViz 卡死
    if (++path_counter_ % 10 == 0) {
      geometry_msgs::PoseStamped ps;
      ps.header = odom.header;
      ps.pose = odom.pose.pose;
      path_.poses.push_back(ps);
      path_.header.stamp = odom.header.stamp;
      pub_path_.publish(path_);
    }
  }

  void publishMap(const ros::WallTimerEvent&) {
    if (pub_map_.getNumSubscribers() == 0) return;   // 没人订阅就别拼了
    lvi::CloudPtr map = system_->buildGlobalMap(0.2);
    if (!map || map->empty()) return;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*map, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_map_.publish(msg);
  }

  lvi::Config cfg_;
  std::unique_ptr<lvi::System> system_;
  std::unique_ptr<lvi::LivoxPreprocess> preprocess_;
  std::vector<lvi::LivoxRawPoint> raw_buf_;

  ros::Subscriber sub_imu_, sub_livox_, sub_img_;
  ros::Publisher pub_odom_, pub_path_, pub_map_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::WallTimer map_timer_;

  nav_msgs::Path path_;
  uint64_t path_counter_ = 0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "lvi_slam");
  ros::NodeHandle nh, pnh("~");

  LviSlamNode node(nh, pnh);

  // 多线程 spinner：IMU 回调频率高，不能被点云/图像回调阻塞。
  // 注意与 ROS2 版的差异：ROS2 默认回调组互斥，ROS1 的 AsyncSpinner 下
  // 不同订阅者的回调会并发，feed* 最终进入 DataSynchronizer，其内部有锁，可并发调用。
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
