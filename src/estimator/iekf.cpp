#include "lvi_slam/estimator/iekf.h"
#include <iostream>

namespace lvi {

IEKF::IEKF(const Options& opt) : opt_(opt) {}

bool IEKF::update(ObservationModel* model, const State& prior, State* state) {
  if (model == nullptr || state == nullptr) return false;

  State  best_state   = *state;
  double last_error   = std::numeric_limits<double>::max();
  int    rematch_num  = 0;
  bool   converged    = false;

  StateMat G = StateMat::Zero();
  const StateMat I = StateMat::Identity();

  for (int iter = 0; iter < opt_.max_iterations; ++iter) {
    // ---- 1. 在当前状态下重新做数据关联并构建残差 ----
    ObservationModel::Residuals res;
    if (!model->buildResidual(*state, &res) || res.num_valid == 0) {
      std::cerr << "[iEKF] " << model->name() << ": no valid observation\n";
      return false;
    }

    // 残差变大说明上一步走过头了，回退并结束。
    // 回退后的状态正是上一轮 G 的线性化点，协方差更新仍然要做——
    // 否则这一帧的观测信息被丢掉，协方差只增不减，几帧后 LIO 权重就趋近于 0。
    if (res.mean_error > last_error) {
      *state = best_state;
      state->cov = (I - G) * state->cov;
      break;
    }
    best_state = *state;
    last_error = res.mean_error;

    const int m = static_cast<int>(res.z.rows());
    const int n = static_cast<int>(res.active_cols.size());

    // ---- 2. 累加 H^T R^-1 H 和 H^T R^-1 z ----
    // 这一步是 O(m * n^2)，线性于观测数，但不涉及大矩阵求逆
    Eigen::MatrixXd HtRinvH = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd HtRinvz = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < m; ++i) {
      const auto  h = res.H_sub.row(i);
      const double w = res.R_inv(i);
      HtRinvH.noalias() += w * h.transpose() * h;
      HtRinvz.noalias() += w * h.transpose() * res.z(i);
    }

    // ---- 3. 展开到全局状态维度 ----
    StateMat H_T_H = StateMat::Zero();
    StateVec H_T_z = StateVec::Zero();
    for (int a = 0; a < n; ++a) {
      const int ga = res.active_cols[a];
      H_T_z(ga) = HtRinvz(a);
      for (int b = 0; b < n; ++b) {
        H_T_H(ga, res.active_cols[b]) = HtRinvH(a, b);
      }
    }

    // ---- 4. 对偶形式卡尔曼增益 ----
    //   K_1 = (H^T R^-1 H + P^-1)^-1      <-- 只对 26x26 求逆
    //   G   = K_1 * H^T R^-1 H
    //   δ   = K_1 * H^T R^-1 z + (prior ⊟ state) - G * (prior ⊟ state)
    //
    // 最后两项是 IEKF 特有的：迭代中状态在移动，但先验固定，
    // 必须把"当前迭代点相对先验的偏移"折算进来。
    const StateMat P_inv = state->cov.inverse();
    const StateMat K_1   = (H_T_H + P_inv).inverse();
    G = K_1 * H_T_H;

    const StateVec vec = prior - *state;               // prior ⊟ state
    const StateVec dx  = K_1 * H_T_z + vec - G * vec;

    *state += dx;

    // ---- 5. 收敛判断 ----
    const double rot_add = dx.segment<3>(kIdxRot).norm() * 57.29578;  // deg
    const double pos_add = dx.segment<3>(kIdxPos).norm() * 100.0;     // cm
    converged = (rot_add < opt_.conv_rot_deg) && (pos_add < opt_.conv_pos_cm);

    // Rematch 机制：一次收敛不算数。状态变了，最近邻关联也该变，
    // 要求"收敛 -> 重新关联 -> 再收敛"至少两轮，确保数据关联稳定。
    if (converged || (rematch_num == 0 && iter == opt_.max_iterations - 2)) {
      ++rematch_num;
    }

    if (rematch_num >= opt_.min_rematch_num || iter == opt_.max_iterations - 1) {
      // ---- 6. 协方差更新，只在最终收敛时做一次（IEKF 标准做法）----
      state->cov = (I - G) * state->cov;
      break;
    }
  }
  return true;
}

IEKF::DegeneracyInfo IEKF::analyzeDegeneracy(const State& state,
                                             double rot_sigma_thresh,
                                             double pos_sigma_thresh) const {
  DegeneracyInfo info;
  const auto& P = state.cov;

  for (int i = 0; i < 6; ++i) {
    info.pose_sigma(i) = std::sqrt(std::max(0.0, P(i, i)));
  }
  for (int i = 0; i < 3; ++i) {
    if (info.pose_sigma(kIdxRot + i) > rot_sigma_thresh) {
      info.is_degenerate = true;
      info.degenerate_dims.push_back(kIdxRot + i);
    }
  }
  for (int i = 0; i < 3; ++i) {
    if (info.pose_sigma(kIdxPos + i) > pos_sigma_thresh) {
      info.is_degenerate = true;
      info.degenerate_dims.push_back(kIdxPos + i);
    }
  }
  return info;
}

}  // namespace lvi
