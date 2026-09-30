// lvi_slam/common/so3_math.h
// SO(3) 上的指数/对数映射与右雅可比
#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace lvi {
namespace so3 {

inline Eigen::Matrix3d Hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m <<     0, -v.z(),  v.y(),
       v.z(),      0, -v.x(),
      -v.y(),  v.x(),      0;
  return m;
}

// 指数映射：so(3) -> SO(3)，Rodrigues 公式
inline Eigen::Matrix3d Exp(const Eigen::Vector3d& phi) {
  const double theta = phi.norm();
  if (theta < 1e-9) {
    // 小角度：一阶近似 + 再正交化，避免 0/0
    return Eigen::Matrix3d::Identity() + Hat(phi);
  }
  const Eigen::Vector3d a = phi / theta;
  const Eigen::Matrix3d A = Hat(a);
  return Eigen::Matrix3d::Identity()
       + std::sin(theta) * A
       + (1.0 - std::cos(theta)) * A * A;
}

inline Eigen::Matrix3d Exp(double vx, double vy, double vz) {
  return Exp(Eigen::Vector3d(vx, vy, vz));
}

// 角速度 * dt 的指数映射（IMU 传播常用）
inline Eigen::Matrix3d Exp(const Eigen::Vector3d& omega, double dt) {
  return Exp(Eigen::Vector3d(omega * dt));
}

// 对数映射：SO(3) -> so(3)
inline Eigen::Vector3d Log(const Eigen::Matrix3d& R) {
  const double cos_theta = std::max(-1.0, std::min(1.0, (R.trace() - 1.0) * 0.5));
  const double theta = std::acos(cos_theta);
  if (theta < 1e-9) {
    return Eigen::Vector3d(R(2, 1) - R(1, 2),
                           R(0, 2) - R(2, 0),
                           R(1, 0) - R(0, 1)) * 0.5;
  }
  const double k = theta / (2.0 * std::sin(theta));
  return Eigen::Vector3d(R(2, 1) - R(1, 2),
                         R(0, 2) - R(2, 0),
                         R(1, 0) - R(0, 1)) * k;
}

// 右雅可比 J_r(phi)，IMU 协方差传播需要
inline Eigen::Matrix3d Jr(const Eigen::Vector3d& phi) {
  const double theta = phi.norm();
  if (theta < 1e-7) return Eigen::Matrix3d::Identity() - 0.5 * Hat(phi);
  const Eigen::Matrix3d A = Hat(phi);
  const double t2 = theta * theta;
  return Eigen::Matrix3d::Identity()
       - (1.0 - std::cos(theta)) / t2 * A
       + (theta - std::sin(theta)) / (t2 * theta) * A * A;
}

// 右雅可比的逆
inline Eigen::Matrix3d JrInv(const Eigen::Vector3d& phi) {
  const double theta = phi.norm();
  if (theta < 1e-7) return Eigen::Matrix3d::Identity() + 0.5 * Hat(phi);
  const Eigen::Matrix3d A = Hat(phi);
  const double half = 0.5 * theta;
  return Eigen::Matrix3d::Identity()
       + 0.5 * A
       + (1.0 / (theta * theta) - (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta))) * A * A;
}

}  // namespace so3
}  // namespace lvi
