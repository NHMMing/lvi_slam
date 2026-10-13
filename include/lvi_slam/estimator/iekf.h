// lvi_slam/estimator/iekf.h
// 迭代误差状态卡尔曼滤波器。
//
// 全系统只有这一个滤波器，LiDAR 和相机都是它的量测更新源。
//
// 核心是对偶形式的卡尔曼增益：
//     K = (H^T R^-1 H + P^-1)^-1 H^T R^-1
// 而不是标准形式 K = P H^T (H P H^T + R)^-1。
//
// 差别在求逆的矩阵尺寸：标准形式要对 (m x m) 求逆（m = 观测数，可达 1e4~1e5），
// 对偶形式只对 (kStateDim x kStateDim) = 26x26 求逆。H^T R^-1 H 的累加是 O(m)
// 的线性开销。这是"整帧点云 + 稠密光度残差"能实时跑的唯一原因。
#pragma once

#include "lvi_slam/common/types.h"
#include <functional>

namespace lvi {

// 观测模型的抽象接口。
// LIO 和 VIO 各自实现它，iEKF 不关心观测来自哪个传感器。
//
// 每次迭代中，iEKF 调用 buildResidual 让观测模型在当前状态下：
//   1. 重新做数据关联（点-面匹配 / patch 重投影）
//   2. 计算残差 z 和雅可比 H
//   3. 计算每个观测的信息量 R^-1（不是全局常数，逐观测从噪声模型推导）
//
// 返回值约定：
//   H_sub:  (m x n_active)  只填有效的状态列，避免存一堆零
//   z:      (m x 1)
//   R_inv:  (m x 1)         对角观测噪声的逆
//   active_cols: H_sub 每一列对应的状态索引
class ObservationModel {
 public:
  virtual ~ObservationModel() = default;

  struct Residuals {
    Eigen::MatrixXd H_sub;              // m x n_active
    Eigen::VectorXd z;                  // m x 1
    Eigen::VectorXd R_inv;              // m x 1
    std::vector<int> active_cols;       // n_active，映射到全局状态索引
    int    num_valid = 0;
    double mean_error = 0.0;
  };

  // 在给定状态下构建残差。返回 false 表示有效观测不足，本次更新应放弃。
  virtual bool buildResidual(const State& state, Residuals* out) = 0;

  // 用于日志和退化分析
  virtual const char* name() const = 0;
};

class IEKF {
 public:
  struct Options {
    int    max_iterations   = 5;
    double conv_rot_deg     = 0.01;   // 旋转增量收敛阈值
    double conv_pos_cm      = 0.015;  // 平移增量收敛阈值
    int    min_rematch_num  = 2;      // 至少重新做 2 次数据关联才认定收敛
  };

  explicit IEKF(const Options& opt);

  // 量测更新。state 既是输入（先验，来自 IMU 传播）也是输出。
  //
  // prior 是 IMU 传播得到的先验状态，迭代中固定不变。
  // 这是 IEKF 相对普通 EKF 的关键：每次迭代的解不是相对上一次迭代的增量，
  // 而是相对 prior 的总增量，所以更新方程里有 (prior ⊟ state) 这一项。
  bool update(ObservationModel* model, const State& prior, State* state);

  // 最近一次 update 的统计，只用于诊断记录
  struct UpdateStats {
    int    iterations = 0;
    int    num_valid  = 0;
    double mean_error = 0.0;
    bool   converged  = false;
  };
  const UpdateStats& lastStats() const { return last_stats_; }

  // 退化分析：返回更新后各状态方向的不确定度增长情况。
  // 与 LIO-SAM 的做法不同——我们看的是协方差而非 Hessian 特征值，
  // 阈值有物理量纲（米 / 弧度），不依赖具体传感器。
  struct DegeneracyInfo {
    bool  is_degenerate = false;
    Eigen::Matrix<double, 6, 1> pose_sigma;   // 位姿 6 个方向的标准差
    std::vector<int> degenerate_dims;         // 退化的维度索引
  };
  DegeneracyInfo analyzeDegeneracy(const State& state,
                                   double rot_sigma_thresh = 0.02,
                                   double pos_sigma_thresh = 0.10) const;

  const Options& options() const { return opt_; }

 private:
  Options opt_;
  UpdateStats last_stats_;
};

}  // namespace lvi
