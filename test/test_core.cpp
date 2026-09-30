// test/test_core.cpp
// 核心数学的单元测试。
//
// 这几个测试是整个系统的安全网。滤波器的 bug 往往表现为「精度差一点」
// 而不是崩溃，没有单测很难发现。
#include <gtest/gtest.h>
#include "lvi_slam/common/types.h"
#include "lvi_slam/common/so3_math.h"

using namespace lvi;

TEST(So3, ExpLogRoundTrip) {
  for (double a = -3.0; a < 3.0; a += 0.37) {
    const V3D phi(a, 0.5 * a, -0.3 * a);
    EXPECT_NEAR((so3::Log(so3::Exp(phi)) - phi).norm(), 0.0, 1e-9);
  }
}

// 小角度分支不能出 NaN —— 这是 sin(θ)/θ 类公式的经典陷阱
TEST(So3, SmallAngleStability) {
  const M3D R = so3::Exp(V3D(1e-12, -1e-12, 1e-13));
  EXPECT_TRUE(R.allFinite());
  EXPECT_NEAR((R - M3D::Identity()).norm(), 0.0, 1e-10);
}

// boxplus / boxminus 必须互逆，否则流形上的迭代会系统性跑偏
TEST(State, BoxplusBoxminusRoundTrip) {
  State a;
  a.rot = so3::Exp(V3D(0.1, -0.2, 0.3));
  a.pos = V3D(1, 2, 3);
  a.ext_rot_ci = so3::Exp(V3D(0.05, 0.02, -0.03));

  StateVec d = StateVec::Random() * 0.01;
  const State b = a.boxplus(d);
  EXPECT_NEAR((b.boxminus(a) - d).norm(), 0.0, 1e-8);
}

// ---------------------------------------------------------------------------
// 对偶形式 vs 标准形式的卡尔曼增益必须等价。
//
// 这个测试是整个效率优化的正确性保证 ——
// 如果两者不等，说明推导错了，而这种错误在实际运行中只表现为「精度差一点」，
// 极难通过看轨迹发现。
// ---------------------------------------------------------------------------
TEST(IEKF, DualFormEqualsStandardForm) {
  const int m = 500;   // 观测数
  const int n = 6;

  Eigen::MatrixXd H = Eigen::MatrixXd::Random(m, n);
  Eigen::VectorXd R_inv = Eigen::VectorXd::Constant(m, 100.0);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(n, n) * 0.01;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m, m) / 100.0;

  // 标准形式：对 (m x m) 求逆 —— m 大时不可行
  const Eigen::MatrixXd K_std =
      P * H.transpose() * (H * P * H.transpose() + R).inverse();
  // 对偶形式：只对 (n x n) 求逆
  const Eigen::MatrixXd K_dual =
      (H.transpose() * R_inv.asDiagonal() * H + P.inverse()).inverse() *
      H.transpose() * R_inv.asDiagonal();

  EXPECT_NEAR((K_std - K_dual).norm() / K_std.norm(), 0.0, 1e-8);
}

// TODO(v1.1): 补充
//  - IMU 传播：无观测时协方差必须单调不减
//  - 体素平面拟合：构造已知平面的带噪点云，验证法向量精度
//  - plane_var：点数增加时必须减小（这是自适应权重的基础）
//  - Scan Context：同一场景旋转后描述子距离应接近 0
