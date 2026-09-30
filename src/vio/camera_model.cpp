// lvi_slam/vio/camera_model.cpp
// 相机模型：针孔 + 畸变。
#include "lvi_slam/vio/camera_model.h"

namespace lvi {

CameraModel::CameraModel(const CameraConfig& cfg) : cfg_(cfg) {
  K_ << cfg.fx, 0, cfg.cx,
        0, cfg.fy, cfg.cy,
        0, 0, 1;
  K_inv_ = K_.inverse();
  buildUndistortMap();
}

V2D CameraModel::world2cam(const V3D& p_c) const {
  const double inv_z = 1.0 / p_c.z();
  return V2D(cfg_.fx * p_c.x() * inv_z + cfg_.cx,
             cfg_.fy * p_c.y() * inv_z + cfg_.cy);
}

V3D CameraModel::cam2world(const V2D& px) const {
  return V3D((px.x() - cfg_.cx) / cfg_.fx,
             (px.y() - cfg_.cy) / cfg_.fy, 1.0).normalized();
}

// ∂[u,v]/∂[x,y,z] —— 直接法和特征点法都要用
void CameraModel::projectionJacobian(const V3D& p_c,
                                     Eigen::Matrix<double, 2, 3>* J) const {
  const double inv_z = 1.0 / p_c.z();
  const double inv_z2 = inv_z * inv_z;
  (*J) << cfg_.fx * inv_z, 0, -cfg_.fx * p_c.x() * inv_z2,
          0, cfg_.fy * inv_z, -cfg_.fy * p_c.y() * inv_z2;
}

bool CameraModel::isInFrame(const V2D& px, int border) const {
  return px.x() >= border && px.y() >= border &&
         px.x() < cfg_.width - border && px.y() < cfg_.height - border;
}

void CameraModel::buildUndistortMap() {
  // 预计算映射表。实时系统一律用 remap 而非 undistort ——
  // 后者每帧重算映射关系，remap 只做查表插值。
  cv::Mat K = (cv::Mat_<double>(3, 3) << cfg_.fx, 0, cfg_.cx,
                                          0, cfg_.fy, cfg_.cy, 0, 0, 1);
  cv::Mat D = (cv::Mat_<double>(1, 4) << cfg_.d0, cfg_.d1, cfg_.d2, cfg_.d3);
  cv::initUndistortRectifyMap(K, D, cv::Mat(), K,
                              cv::Size(cfg_.width, cfg_.height),
                              CV_32FC1, map_x_, map_y_);
}

void CameraModel::undistortImage(const cv::Mat& src, cv::Mat* dst) const {
  cv::remap(src, *dst, map_x_, map_y_, cv::INTER_LINEAR);
}

}  // namespace lvi
