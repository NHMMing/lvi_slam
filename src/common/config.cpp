// lvi_slam/common/config.cpp
// YAML 参数加载。所有魔法数字集中在配置文件，代码里不留硬编码阈值。
#include "lvi_slam/common/config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace lvi {
namespace {

template <typename T>
T get(const YAML::Node& n, const char* key, const T& def) {
  return (n && n[key]) ? n[key].as<T>() : def;
}

// 从 9 元素数组读旋转矩阵（行主序），从 3 元素读平移
bool readMat3(const YAML::Node& n, const char* key, Eigen::Matrix3d* out) {
  if (!n || !n[key]) return false;
  const auto v = n[key].as<std::vector<double>>();
  if (v.size() != 9) return false;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) (*out)(i, j) = v[i * 3 + j];
  return true;
}

bool readVec3(const YAML::Node& n, const char* key, Eigen::Vector3d* out) {
  if (!n || !n[key]) return false;
  const auto v = n[key].as<std::vector<double>>();
  if (v.size() != 3) return false;
  *out = Eigen::Vector3d(v[0], v[1], v[2]);
  return true;
}

}  // namespace

bool Config::loadFromFile(const std::string& path) {
  try {
    const YAML::Node root = YAML::LoadFile(path);

    lidar_topic = get<std::string>(root, "lidar_topic", lidar_topic);
    imu_topic   = get<std::string>(root, "imu_topic",   imu_topic);
    image_topic = get<std::string>(root, "image_topic", image_topic);
    max_iterations = get<int>(root, "max_iterations", max_iterations);
    num_threads    = get<int>(root, "num_threads",    num_threads);

    if (const auto n = root["imu"]) {
      imu.acc_noise      = get<double>(n, "acc_noise",      imu.acc_noise);
      imu.gyr_noise      = get<double>(n, "gyr_noise",      imu.gyr_noise);
      imu.acc_bias_noise = get<double>(n, "acc_bias_noise", imu.acc_bias_noise);
      imu.gyr_bias_noise = get<double>(n, "gyr_bias_noise", imu.gyr_bias_noise);
      imu.gravity_norm   = get<double>(n, "gravity_norm",   imu.gravity_norm);
      imu.init_frame_num = get<int>(n, "init_frame_num",    imu.init_frame_num);
    }

    if (const auto n = root["lidar"]) {
      lidar.depth_err   = get<double>(n, "depth_err",   lidar.depth_err);
      lidar.beam_err    = get<double>(n, "beam_err",    lidar.beam_err);
      lidar.blind_min   = get<double>(n, "blind_min",   lidar.blind_min);
      lidar.blind_max   = get<double>(n, "blind_max",   lidar.blind_max);
      lidar.filter_size = get<double>(n, "filter_size", lidar.filter_size);
      lidar.point_filter_num = get<int>(n, "point_filter_num", lidar.point_filter_num);
      lidar.use_livox_tag    = get<bool>(n, "use_livox_tag",  lidar.use_livox_tag);
      readMat3(n, "ext_rot_li", &lidar.ext_rot_li);
      readVec3(n, "ext_pos_li", &lidar.ext_pos_li);
    }

    if (const auto n = root["voxel"]) {
      voxel.max_voxel_size  = get<double>(n, "max_voxel_size",  voxel.max_voxel_size);
      voxel.max_layer       = get<int>(n, "max_layer",          voxel.max_layer);
      voxel.max_points_num  = get<int>(n, "max_points_num",     voxel.max_points_num);
      voxel.plane_threshold = get<double>(n, "plane_threshold", voxel.plane_threshold);
      voxel.sigma_num       = get<double>(n, "sigma_num",       voxel.sigma_num);
      voxel.sliding_en      = get<bool>(n, "sliding_en",        voxel.sliding_en);
      voxel.sliding_thresh  = get<double>(n, "sliding_thresh",  voxel.sliding_thresh);
      voxel.half_map_size   = get<int>(n, "half_map_size",      voxel.half_map_size);
    }

    if (const auto n = root["visual"]) {
      visual.enable        = get<bool>(n, "enable",        visual.enable);
      visual.patch_size    = get<int>(n, "patch_size",     visual.patch_size);
      visual.pyramid_level = get<int>(n, "pyramid_level",  visual.pyramid_level);
      visual.grid_size     = get<int>(n, "grid_size",      visual.grid_size);
      visual.outlier_thresh= get<double>(n, "outlier_thresh", visual.outlier_thresh);
      visual.img_point_cov = get<double>(n, "img_point_cov",  visual.img_point_cov);
      visual.estimate_exposure = get<bool>(n, "estimate_exposure", visual.estimate_exposure);
      visual.estimate_td       = get<bool>(n, "estimate_td",       visual.estimate_td);
      visual.use_plane_normal  = get<bool>(n, "use_plane_normal",  visual.use_plane_normal);
      visual.hybrid_mode       = get<bool>(n, "hybrid_mode",       visual.hybrid_mode);
      visual.photometric_health_thresh =
          get<double>(n, "photometric_health_thresh", visual.photometric_health_thresh);
      visual.width  = get<int>(n, "width",  visual.width);
      visual.height = get<int>(n, "height", visual.height);
      visual.fx = get<double>(n, "fx", visual.fx);
      visual.fy = get<double>(n, "fy", visual.fy);
      visual.cx = get<double>(n, "cx", visual.cx);
      visual.cy = get<double>(n, "cy", visual.cy);
      visual.k1 = get<double>(n, "k1", visual.k1);
      visual.k2 = get<double>(n, "k2", visual.k2);
      visual.p1 = get<double>(n, "p1", visual.p1);
      visual.p2 = get<double>(n, "p2", visual.p2);
    }

    if (const auto n = root["extrinsic"]) {
      readMat3(n, "ext_rot_ci", &extrinsic.ext_rot_ci);
      readVec3(n, "ext_pos_ci", &extrinsic.ext_pos_ci);
      extrinsic.online_calib_level = get<int>(n, "online_calib_level",
                                              extrinsic.online_calib_level);
      extrinsic.unlock_rot_after_sec = get<double>(n, "unlock_rot_after_sec",
                                                   extrinsic.unlock_rot_after_sec);
      extrinsic.unlock_pos_after_sec = get<double>(n, "unlock_pos_after_sec",
                                                   extrinsic.unlock_pos_after_sec);
    }

    if (const auto n = root["keyframe"]) {
      keyframe.dist_thresh    = get<double>(n, "dist_thresh",    keyframe.dist_thresh);
      keyframe.angle_thresh   = get<double>(n, "angle_thresh",   keyframe.angle_thresh);
      keyframe.time_thresh    = get<double>(n, "time_thresh",    keyframe.time_thresh);
      keyframe.cov_thresh     = get<double>(n, "cov_thresh",     keyframe.cov_thresh);
      keyframe.loop_kf_stride = get<double>(n, "loop_kf_stride", keyframe.loop_kf_stride);
    }

    if (const auto n = root["backend"]) {
      backend.enable             = get<bool>(n, "enable", backend.enable);
      backend.sc_dist_thresh     = get<double>(n, "sc_dist_thresh",     backend.sc_dist_thresh);
      backend.loop_min_time_diff = get<double>(n, "loop_min_time_diff", backend.loop_min_time_diff);
      backend.icp_fitness_thresh = get<double>(n, "icp_fitness_thresh", backend.icp_fitness_thresh);
      backend.icp_max_iter       = get<int>(n, "icp_max_iter",          backend.icp_max_iter);
      backend.dual_modal_verify  = get<bool>(n, "dual_modal_verify",    backend.dual_modal_verify);
      backend.use_gps            = get<bool>(n, "use_gps",              backend.use_gps);
      backend.gps_cov_thresh     = get<double>(n, "gps_cov_thresh",     backend.gps_cov_thresh);
      backend.pose_cov_thresh    = get<double>(n, "pose_cov_thresh",    backend.pose_cov_thresh);
      backend.gps_min_stride     = get<double>(n, "gps_min_stride",     backend.gps_min_stride);
      backend.use_gps_z          = get<bool>(n, "use_gps_z",            backend.use_gps_z);
      backend.relocal_kf_num     = get<int>(n, "relocal_kf_num",        backend.relocal_kf_num);
    }

    return validate();
  } catch (const std::exception& e) {
    std::cerr << "[Config] 加载失败: " << e.what() << std::endl;
    return false;
  }
}

// 启动时就把参数错误暴露出来，而不是跑到一半才发现。
bool Config::validate() const {
  auto fail = [](const char* msg) { std::cerr << "[Config] " << msg << std::endl; return false; };

  if (voxel.max_voxel_size <= 0)   return fail("voxel.max_voxel_size 必须为正");
  if (voxel.max_layer < 0 || voxel.max_layer > 6) return fail("voxel.max_layer 应在 0~6");
  if (imu.init_frame_num < 10)     return fail("imu.init_frame_num 过少，静止初始化会不准");
  if (lidar.depth_err <= 0 || lidar.beam_err <= 0)
    return fail("lidar 噪声参数必须为正 —— 观测权重完全由它们推导");
  if (lidar.blind_min >= lidar.blind_max) return fail("lidar blind 区间非法");
  if (lidar.point_filter_num < 1)  return fail("point_filter_num 必须 >= 1");

  if (visual.enable) {
    if (visual.fx <= 0 || visual.fy <= 0) return fail("启用视觉但相机内参未设");
    if (visual.patch_size % 2 != 0)       return fail("patch_size 必须为偶数");
    if (visual.pyramid_level < 1)         return fail("pyramid_level 必须 >= 1");
  }
  if (num_threads < 1) return fail("num_threads 必须 >= 1");
  return true;
}

}  // namespace lvi
