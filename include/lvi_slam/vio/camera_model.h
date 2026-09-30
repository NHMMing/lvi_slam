// lvi_slam/vio/camera_model.h
#pragma once
#include "lvi_slam/common/types.h"
#include "lvi_slam/common/config.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace lvi {

class CameraModel {
 public:
  explicit CameraModel(const CameraConfig& cfg);

  V2D world2cam(const V3D& p_c) const;
  V3D cam2world(const V2D& px) const;
  void projectionJacobian(const V3D& p_c, Eigen::Matrix<double, 2, 3>* J) const;
  bool isInFrame(const V2D& px, int border) const;
  void undistortImage(const cv::Mat& src, cv::Mat* dst) const;

  int width()  const { return cfg_.width; }
  int height() const { return cfg_.height; }

 private:
  void buildUndistortMap();

  CameraConfig cfg_;
  M3D K_, K_inv_;
  cv::Mat map_x_, map_y_;   // 预计算的去畸变映射表，实时系统必须用 remap
};

}  // namespace lvi
