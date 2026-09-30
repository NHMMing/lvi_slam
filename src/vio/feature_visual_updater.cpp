// lvi_slam/vio/feature_visual_updater.cpp
// 特征点法备轨。
//
// 两个用途，缺一不可：
//   1. 直接法失效时（光照突变）接管位姿估计
//   2. 即使正常时也低频运行 —— 直接法没有描述子，做不了视觉回环，
//      这一轨为后端的双模态回环验证积累 ORB 描述子
//
// 与 VINS-Mono 的关键简化：深度不三角化，直接从 LiDAR 点云投影获得。
// 省掉了逆深度参数化、滑窗内深度优化、视差判断一整套逻辑。
#include "lvi_slam/vio/vio_updater.h"

#include <opencv2/features2d.hpp>

namespace lvi {

FeatureVisualUpdater::FeatureVisualUpdater(const Config& cfg, VoxelMap* map,
                                           CameraModel* cam)
    : cfg_(cfg), map_(map), cam_(cam) {
  // ORB 而非 SIFT/SURF：二进制描述子，匹配快，且回环用的 BoW 词典也是 ORB 的
  orb_ = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
}

void FeatureVisualUpdater::setFrame(const cv::Mat& gray, double timestamp) {
  img_gray_ = gray;
  timestamp_ = timestamp;
  keypoints_.clear();
  points_3d_.clear();
  has_depth_.clear();
  orb_->detectAndCompute(gray, cv::noArray(), keypoints_, descriptors_);
}

// 从 LiDAR 点云给特征点赋深度。
//
// 做法是把点云投影到图像，为每个特征点在其邻域内找最近的投影点。
// 比三角化简单得多，而且精度高一个量级 —— 这是有 LiDAR 的直接收益。
void FeatureVisualUpdater::associateDepthFromLidar(
    const State& state, const std::vector<PointWithVar>& lidar_points) {
  const size_t n = keypoints_.size();
  points_3d_.assign(n, V3D::Zero());
  has_depth_.assign(n, false);
  if (n == 0 || lidar_points.empty()) return;

  const M3D R_cw = state.rotWorldCam().transpose();
  const V3D p_cw = -R_cw * state.posWorldCam();

  // 用一张深度图做空间索引，避免对每个特征点遍历全部点云
  const int W = cfg_.visual.width, H = cfg_.visual.height;
  cv::Mat depth_map(H, W, CV_32F, cv::Scalar(0));
  std::vector<V3D> idx_map(W * H, V3D::Zero());

  for (const auto& pv : lidar_points) {
    const V3D p_c = R_cw * pv.point_w + p_cw;
    if (p_c.z() < 0.5) continue;
    const V2D px = cam_->world2cam(p_c);
    const int u = static_cast<int>(px.x()), v = static_cast<int>(px.y());
    if (u < 0 || v < 0 || u >= W || v >= H) continue;

    const float d = static_cast<float>(p_c.z());
    float& cur = depth_map.at<float>(v, u);
    // 同一像素有多个点时取最近的 —— 远处的点被前景遮挡，不可见
    if (cur < 1e-3f || d < cur) {
      cur = d;
      idx_map[v * W + u] = pv.point_w;
    }
  }

  constexpr int kSearchRadius = 3;
  for (size_t i = 0; i < n; ++i) {
    const int u0 = static_cast<int>(keypoints_[i].pt.x);
    const int v0 = static_cast<int>(keypoints_[i].pt.y);
    float best_d = 1e9f;
    V3D best_p = V3D::Zero();

    for (int dv = -kSearchRadius; dv <= kSearchRadius; ++dv) {
      for (int du = -kSearchRadius; du <= kSearchRadius; ++du) {
        const int u = u0 + du, v = v0 + dv;
        if (u < 0 || v < 0 || u >= W || v >= H) continue;
        const float d = depth_map.at<float>(v, u);
        if (d > 1e-3f && d < best_d) { best_d = d; best_p = idx_map[v * W + u]; }
      }
    }
    if (best_d < 1e8f) { points_3d_[i] = best_p; has_depth_[i] = true; }
  }
}

// 重投影误差。
// 深度是已知量（LiDAR 给的），不参与优化 —— 这比 VINS-Mono 简单，
// 它要同时优化每个特征的逆深度。
bool FeatureVisualUpdater::buildResidual(const State& state, Residuals* out) {
  std::vector<int> valid_idx;
  for (size_t i = 0; i < keypoints_.size(); ++i) {
    if (has_depth_[i]) valid_idx.push_back(static_cast<int>(i));
  }
  if (static_cast<int>(valid_idx.size()) < 15) return false;

  const int m = static_cast<int>(valid_idx.size()) * 2;
  out->active_cols.clear();
  for (int i = 0; i < 3; ++i) out->active_cols.push_back(kIdxRot + i);
  for (int i = 0; i < 3; ++i) out->active_cols.push_back(kIdxPos + i);

  out->H_sub.setZero(m, 6);
  out->z.setZero(m);
  out->R_inv.setZero(m);

  const M3D R_cw = state.rotWorldCam().transpose();
  const V3D p_cw = -R_cw * state.posWorldCam();
  const M3D R_ci = state.ext_rot_ci;

  int valid = 0;
  for (size_t k = 0; k < valid_idx.size(); ++k) {
    const int i = valid_idx[k];
    const V3D p_c = R_cw * points_3d_[i] + p_cw;
    if (p_c.z() < 0.1) continue;

    const V2D proj = cam_->world2cam(p_c);
    const V2D obs(keypoints_[i].pt.x, keypoints_[i].pt.y);
    const V2D res = proj - obs;

    // 简易外点剔除：重投影误差过大说明深度关联错了（打在了遮挡边缘）
    if (res.norm() > 10.0) continue;

    Eigen::Matrix<double, 2, 3> J_proj;
    cam_->projectionJacobian(p_c, &J_proj);
    const M3D p_hat = so3::Hat(p_c);

    out->z.segment<2>(k * 2) = res;
    out->H_sub.block<2, 3>(k * 2, 0) = (J_proj * p_hat) * R_ci;
    out->H_sub.block<2, 3>(k * 2, 3) = (-J_proj) * R_cw;
    // 像素观测噪声约 1.5 px，按金字塔层放大
    const double sigma_px = 1.5 * (1 << keypoints_[i].octave);
    out->R_inv.segment<2>(k * 2).setConstant(1.0 / (sigma_px * sigma_px));
    ++valid;
  }

  out->num_valid = valid;
  return valid >= 15;
}

}  // namespace lvi
