// lvi_slam/common/types.h
// 核心类型定义：状态流形、点云类型、观测包
#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <memory>
#include <deque>

#include "lvi_slam/common/so3_math.h"

namespace lvi {

// ---------------------------------------------------------------------------
// 状态维度定义
//
// 索引布局（error-state，切空间维度）：
//   [0:3)   rot        旋转（SO(3) 切空间）
//   [3:6)   pos        位置
//   [6:9)   vel        速度
//   [9:12)  bg         陀螺 bias
//   [12:15) ba         加速度计 bias
//   [15:18) grav       重力向量
//   [18]    inv_expo   逆曝光时间（视觉直接法用）
//   [19]    td         相机-IMU 时间偏移（在线估计）
//   [20:23) ext_rot_ci 相机-IMU 外参旋转
//   [23:26) ext_pos_ci 相机-IMU 外参平移
//
// 设计说明：
//   LiDAR-IMU 外参不进状态向量。原因是 LiDAR 观测对外参的可观性弱于相机，
//   且实践中 LiDAR-IMU 刚性安装更可靠。如需在线标定，扩展到 32 维即可。
// ---------------------------------------------------------------------------
constexpr int kStateDim = 26;

constexpr int kIdxRot     = 0;
constexpr int kIdxPos     = 3;
constexpr int kIdxVel     = 6;
constexpr int kIdxBg      = 9;
constexpr int kIdxBa      = 12;
constexpr int kIdxGrav    = 15;
constexpr int kIdxInvExpo = 18;
constexpr int kIdxTd      = 19;
constexpr int kIdxExtRot  = 20;
constexpr int kIdxExtPos  = 23;

using V2D = Eigen::Vector2d;
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;
using StateVec = Eigen::Matrix<double, kStateDim, 1>;
using StateMat = Eigen::Matrix<double, kStateDim, kStateDim>;

using PointType  = pcl::PointXYZINormal;   // normal 字段复用为时间偏移/曲率
using PointCloud = pcl::PointCloud<PointType>;
using CloudPtr   = PointCloud::Ptr;

// ---------------------------------------------------------------------------
// State：流形上的系统状态
//
// 关键约定：
//   - 旋转量用矩阵存储，更新用右乘扰动 R <- R * Exp(dphi)
//   - operator+ / operator- 是流形上的广义加减（boxplus / boxminus）
//   - 协方差存的是 error-state 的协方差，维度 kStateDim
// ---------------------------------------------------------------------------
struct State {
  M3D    rot      = M3D::Identity();      // R_world_imu
  V3D    pos      = V3D::Zero();          // t_world_imu
  V3D    vel      = V3D::Zero();          // 世界系速度
  V3D    bg       = V3D::Zero();
  V3D    ba       = V3D::Zero();
  V3D    grav     = V3D(0, 0, -9.81);     // 世界系重力
  double inv_expo = 1.0;                  // 逆曝光时间
  double td       = 0.0;                  // 相机时间偏移（秒）
  M3D    ext_rot_ci = M3D::Identity();    // R_camera_imu
  V3D    ext_pos_ci = V3D::Zero();        // t_camera_imu

  StateMat cov = StateMat::Identity() * 1e-3;

  double timestamp = -1.0;

  // 广义加法：x ⊞ δ
  State boxplus(const StateVec& d) const {
    State s = *this;
    s.rot        = rot * so3::Exp(d.segment<3>(kIdxRot));
    s.pos       += d.segment<3>(kIdxPos);
    s.vel       += d.segment<3>(kIdxVel);
    s.bg        += d.segment<3>(kIdxBg);
    s.ba        += d.segment<3>(kIdxBa);
    s.grav      += d.segment<3>(kIdxGrav);
    s.inv_expo  += d(kIdxInvExpo);
    s.td        += d(kIdxTd);
    s.ext_rot_ci = ext_rot_ci * so3::Exp(d.segment<3>(kIdxExtRot));
    s.ext_pos_ci+= d.segment<3>(kIdxExtPos);
    return s;
  }

  State& operator+=(const StateVec& d) { *this = boxplus(d); return *this; }

  // 广义减法：this ⊟ other
  StateVec boxminus(const State& o) const {
    StateVec d = StateVec::Zero();
    d.segment<3>(kIdxRot)    = so3::Log(o.rot.transpose() * rot);
    d.segment<3>(kIdxPos)    = pos  - o.pos;
    d.segment<3>(kIdxVel)    = vel  - o.vel;
    d.segment<3>(kIdxBg)     = bg   - o.bg;
    d.segment<3>(kIdxBa)     = ba   - o.ba;
    d.segment<3>(kIdxGrav)   = grav - o.grav;
    d(kIdxInvExpo)           = inv_expo - o.inv_expo;
    d(kIdxTd)                = td   - o.td;
    d.segment<3>(kIdxExtRot) = so3::Log(o.ext_rot_ci.transpose() * ext_rot_ci);
    d.segment<3>(kIdxExtPos) = ext_pos_ci - o.ext_pos_ci;
    return d;
  }

  StateVec operator-(const State& o) const { return boxminus(o); }

  // 世界系 <- 相机系
  M3D rotWorldCam()  const { return rot * ext_rot_ci.transpose(); }
  V3D posWorldCam()  const { return pos - rotWorldCam() * ext_pos_ci; }
};

// ---------------------------------------------------------------------------
// 传感器原始数据
// ---------------------------------------------------------------------------
struct ImuData {
  double t = 0;
  V3D acc  = V3D::Zero();
  V3D gyr  = V3D::Zero();
};

struct ImageData {
  double  t = 0;
  cv::Mat gray;    // 灰度图（直接法用）
  cv::Mat rgb;     // 彩色图（着色/可视化用，可空）
};

// 同步后的观测包，前端每轮消费一个
enum class UpdateMode { kNone, kLio, kVio };

struct SyncPackage {
  UpdateMode      mode = UpdateMode::kNone;
  double          sync_time = -1.0;   // 本轮更新的目标时刻
  double          last_update_time = -1.0;
  std::vector<ImuData> imus;          // (last_update_time, sync_time] 的 IMU
  CloudPtr        cloud;              // 已按 sync_time 切分好的点云
  ImageData       image;              // mode==kVio 时有效
};

// ---------------------------------------------------------------------------
// 带不确定度的点（LIO 残差构建的基本单元）
// ---------------------------------------------------------------------------
struct PointWithVar {
  V3D point_b = V3D::Zero();   // body(IMU) 系
  V3D point_w = V3D::Zero();   // world 系
  M3D var     = M3D::Zero();   // world 系协方差
  M3D body_var= M3D::Zero();   // body 系协方差（传感器噪声模型）
};

// 点到平面的匹配结果
struct PointToPlane {
  V3D    point_b   = V3D::Zero();
  V3D    point_w   = V3D::Zero();
  V3D    normal    = V3D::Zero();
  V3D    center    = V3D::Zero();
  M3D    body_cov  = M3D::Zero();
  Eigen::Matrix<double, 6, 6> plane_var =
      Eigen::Matrix<double, 6, 6>::Zero();   // 平面参数(n,q)的协方差
  double dist      = 0.0;                    // 点到平面距离（有符号）
  bool   valid     = false;
};

// ---------------------------------------------------------------------------
// 关键帧：前端产出，后端消费
// ---------------------------------------------------------------------------
struct Keyframe {
  int      id   = -1;
  double   t    = -1.0;
  M3D      rot  = M3D::Identity();     // 前端估计的世界位姿
  V3D      pos  = V3D::Zero();
  Eigen::Matrix<double, 6, 6> pose_cov =
      Eigen::Matrix<double, 6, 6>::Identity();  // 前端给出的位姿协方差

  CloudPtr cloud_body;                 // body 系降采样点云（回环 ICP 用）
  cv::Mat  descriptors;                // 视觉描述子（视觉回环用，可空）
  Eigen::MatrixXf scan_context;        // Scan Context 描述子

  bool     is_loop_candidate = false;  // 是否进入回环检索库
};
using KeyframePtr = std::shared_ptr<Keyframe>;

}  // namespace lvi
