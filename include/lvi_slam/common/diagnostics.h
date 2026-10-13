// lvi_slam/common/diagnostics.h
// 评估用诊断记录：逐帧滤波器健康度 + 回环事件。
//
// 只做记录，不参与估计。默认关闭（System::enableDiagnostics），
// 在线节点不受影响；离线运行器打开后写成 CSV，交给 tools/eval 分析。
//
// 字段约定（与 tools/eval 的读取代码一一对应，改这里要同步改那边）：
//   pose_cov  error-state 的位姿协方差，顺序 [rot(3), pos(3)]；
//             旋转是右扰动 R = R̂·Exp(δθ)（body 系），平移在世界系。
#pragma once

#include "lvi_slam/common/types.h"

namespace lvi {

struct FrameDiag {
  double t = -1.0;
  int    mode = 0;            // 1 = LIO, 2 = VIO
  bool   ok = false;          // 本帧量测更新是否成功
  double proc_ms = 0.0;       // 前端处理本帧的耗时

  // iEKF 本次更新
  int    iterations = 0;
  int    num_valid = 0;       // 有效残差数
  double mean_error = 0.0;    // 最终迭代的平均残差
  bool   converged = false;

  int    degenerate_mask = 0; // 第 i 位 = 位姿第 i 维退化（0-2 旋转，3-5 平移）
  int    vio_track = -1;      // -1 无视觉，0 直接法，1 特征法

  Eigen::Matrix<double, 6, 6> pose_cov = Eigen::Matrix<double, 6, 6>::Zero();
  double td = 0.0;
  V3D    bg = V3D::Zero();
  V3D    ba = V3D::Zero();
};

// 回环候选在哪一级被拒绝 / 接受。只记录通过 Scan Context 阈值的候选——
// 每个关键帧都会查一次库，SC 级别的拒绝没有信息量。
enum class LoopStage : int {
  kRejectedIcp    = 1,   // SC 命中但 ICP 未收敛 / fitness 超阈值 / 点数不足
  kRejectedVisual = 2,   // ICP 通过但视觉交叉验证拒绝
  kAccepted       = 3,   // 通过全部验证，进入因子图
};

struct LoopEvent {
  LoopStage stage = LoopStage::kRejectedIcp;
  int    id_from = -1, id_to = -1;     // 当前关键帧 / 历史关键帧
  double t_from = -1.0, t_to = -1.0;
  double sc_dist = -1.0;
  double icp_fitness = -1.0;           // 未跑到 ICP 时为 -1
  // 测得的相对位姿 T_to_from（历史帧系下看当前帧），用于对照真值判断真/假回环
  M3D    rot_delta = M3D::Identity();
  V3D    pos_delta = V3D::Zero();
};

}  // namespace lvi
