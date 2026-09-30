// lvi_slam/vio/visual_types.h
#pragma once

#include "lvi_slam/common/types.h"
#include <opencv2/core.hpp>

namespace lvi {

// 一个 3D 点在某一帧中的图像块观测
struct VisualPatch {
  int    id = -1;
  double timestamp = 0.0;

  M3D    rot_cw = M3D::Identity();   // 该观测帧的相机位姿 (world -> cam)
  V3D    pos_cw = V3D::Zero();

  V2D    px     = V2D::Zero();       // 像素坐标
  V3D    bearing= V3D::Zero();       // 归一化方向向量

  std::vector<float> patch;          // 多层金字塔的像素值
  float  mean       = 0.f;           // patch 均值（NCC 用，惰性计算）
  float  score      = 0.f;           // 参考 patch 优选打分
  double inv_expo   = 1.0;           // 该帧的逆曝光时间
  int    pyr_level  = 0;
};
using VisualPatchPtr = std::shared_ptr<VisualPatch>;

// 当前帧的视觉子地图：本轮参与光度更新的所有点及其参考 patch
struct VisualSubMap {
  std::vector<VisualPointPtr>     points;
  std::vector<std::vector<float>> warped_patches;  // 已做仿射变换的参考 patch
  std::vector<int>                search_levels;   // 每个点的金字塔层
  std::vector<double>             ref_inv_expos;
  std::vector<float>              errors;
  std::vector<float>              propa_errors;    // 更新前的残差（用于外点判定）

  void clear() {
    points.clear(); warped_patches.clear(); search_levels.clear();
    ref_inv_expos.clear(); errors.clear(); propa_errors.clear();
  }
  size_t size() const { return points.size(); }
};

// 光度健康度：混合前端切换轨道的依据
struct PhotometricHealth {
  double median_residual   = 0.0;
  double history_baseline  = 0.0;
  double ratio             = 1.0;
  bool   is_healthy        = true;
  int    valid_point_num   = 0;
};

}  // namespace lvi
