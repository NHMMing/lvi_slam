// lvi_slam/common/config.h
// 全局配置。所有魔法数字集中在此，禁止散落在代码里。
#pragma once

#include <string>
#include <Eigen/Dense>

namespace lvi {

struct ImuConfig {
  double acc_noise      = 1e-2;   // m/s^2/sqrt(Hz)
  double gyr_noise      = 1e-3;   // rad/s/sqrt(Hz)
  double acc_bias_noise = 1e-4;   // 随机游走
  double gyr_bias_noise = 1e-5;
  double gravity_norm   = 9.81;
  int    init_frame_num = 20;     // 静止初始化用的帧数
};

struct LidarConfig {
  // 传感器噪声模型：LIO 观测权重从这两个量推导，不做手工调参
  double depth_err = 0.02;        // 沿光束方向测距误差 (m)
  double beam_err  = 0.05;        // 角分辨率误差 (deg)

  double blind_min = 0.5;         // 盲区最小距离
  double blind_max = 100.0;
  double filter_size = 0.2;       // 体素降采样边长

  // ---- Livox 专有 ----
  int    point_filter_num = 3;    // 每 N 点取 1（Mid-360 单帧约 2 万点）
  bool   use_livox_tag    = true; // 用 tag 字段剔除雨雾/拖点噪声

  // LiDAR-IMU 外参（离线标定给定）
  Eigen::Matrix3d ext_rot_li = Eigen::Matrix3d::Identity();
  Eigen::Vector3d ext_pos_li = Eigen::Vector3d::Zero();
};

struct VoxelMapConfig {
  double max_voxel_size   = 0.5;
  int    max_layer        = 3;      // 八叉树最大层数
  int    max_points_num   = 50;     // 单体素最大点数
  double plane_threshold  = 0.01;   // 最小特征值阈值，判定是否为平面
  double sigma_num        = 3.0;    // 残差卡方检验倍数

  // 局部地图滑窗（内存有界的关键）
  bool   sliding_en    = true;
  double sliding_thresh= 8.0;       // 移动多远触发一次滑窗
  int    half_map_size = 100;       // 保留半径（体素数）
};

struct CameraConfig {
  int    width = 640, height = 480;
  double fx = 400, fy = 400, cx = 320, cy = 240;
  double d0 = 0, d1 = 0, d2 = 0, d3 = 0;   // 畸变系数 (k1, k2, p1, p2)
};

struct VisualConfig {
  bool   enable = true;

  int    patch_size   = 8;
  int    pyramid_level= 3;
  int    grid_size    = 40;         // 特征分布网格边长（像素）
  double outlier_thresh = 300.0;    // 光度残差外点阈值
  double img_point_cov  = 100.0;    // 光度观测噪声

  bool   estimate_exposure = true;  // 逆曝光时间在线估计
  bool   estimate_td       = true;  // 时间偏移在线估计
  bool   use_plane_normal  = true;  // 用 LiDAR 法向量算精确单应

  // 混合前端：直接法为主，特征点法为备
  bool   hybrid_mode = true;
  double photometric_health_thresh = 3.0;  // 残差中位数超过历史 N 倍则切备轨

  // 相机内参
  int    width = 640, height = 480;
  double fx = 400, fy = 400, cx = 320, cy = 240;
  double k1 = 0, k2 = 0, p1 = 0, p2 = 0;
};

struct ExtrinsicConfig {
  // 相机-IMU 外参初值（离线标定，如 Kalibr）
  Eigen::Matrix3d ext_rot_ci = Eigen::Matrix3d::Identity();
  Eigen::Vector3d ext_pos_ci = Eigen::Vector3d::Zero();

  // 在线标定的分阶段解锁策略
  // 0: 全程固定  1: 运动充分后解锁旋转  2: 旋转收敛后再解锁平移
  int  online_calib_level = 1;
  double unlock_rot_after_sec = 10.0;
  double unlock_pos_after_sec = 60.0;
};

struct KeyframeConfig {
  double dist_thresh   = 1.0;    // m
  double angle_thresh  = 0.2;    // rad
  double time_thresh   = 2.0;    // s，静止时的兜底（防位姿图断裂）
  double cov_thresh    = 1.0;    // 协方差过大的帧不做关键帧

  double loop_kf_stride = 3.0;   // 每隔多远取一个进回环检索库
};

struct BackendConfig {
  bool   enable = true;

  // 回环检测
  double sc_dist_thresh      = 0.35;   // Scan Context 相似度阈值
  double loop_min_time_diff  = 30.0;   // 秒，排除相邻帧假回环
  double icp_fitness_thresh  = 0.3;
  int    icp_max_iter        = 100;
  bool   dual_modal_verify   = true;   // 激光+视觉双模态验证回环

  // GPS（可选）
  bool   use_gps         = false;
  double gps_cov_thresh  = 2.0;
  double pose_cov_thresh = 0.05;       // 位姿不确定度超过才用 GPS
  double gps_min_stride  = 5.0;
  bool   use_gps_z       = false;

  // 回环后前端局部地图重建的关键帧数量
  int    relocal_kf_num = 20;
};

struct Config {
  ImuConfig       imu;
  LidarConfig     lidar;
  VoxelMapConfig  voxel;
  VisualConfig    visual;
  ExtrinsicConfig extrinsic;
  KeyframeConfig  keyframe;
  BackendConfig   backend;

  int    max_iterations = 5;      // iEKF 最大迭代次数
  int    num_threads    = 4;

  std::string lidar_topic = "/lidar_points";
  std::string imu_topic   = "/imu";
  std::string image_topic = "/camera/image_raw";

  // 从 YAML 加载
  bool loadFromFile(const std::string& path);
  bool validate() const;
};

}  // namespace lvi
