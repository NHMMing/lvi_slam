// lvi_slam/preprocess/livox_preprocess.h
// Livox 点云预处理。
//
// Livox 与机械式雷达（Velodyne/Ouster）的三个本质差异，决定了预处理必须单独写：
//
//   1. 非重复扫描。点不按扫描线规律排列，没有稳定的 ring 结构，
//      LOAM 那套「沿 ring 算曲率」的特征提取完全不适用。
//      —— 本系统不提特征（整帧点云进 iEKF），所以这一点反而不是问题。
//
//   2. 每个点自带纳秒级 offset_time 和 line 编号。时间戳精度比机械式更高，
//      去畸变可以做得更准。
//
//   3. 有 tag 字段标记回波质量和噪点。这是 Livox 独有的先验，
//      不用白不用 —— 能省掉很多后续的外点处理。
#pragma once

#include "lvi_slam/common/types.h"
#include "lvi_slam/common/config.h"

namespace lvi {

// Livox 自定义点的最小抽象。
// 这样核心库不依赖 livox_ros_driver2 的头文件，ROS 层负责转换。
struct LivoxRawPoint {
  float    x = 0, y = 0, z = 0;
  float    reflectivity = 0;
  uint8_t  tag  = 0;      // bit[5:4] 回波置信度, bit[3:2] 粗噪点, bit[1:0] 雨雾噪点
  uint8_t  line = 0;      // 激光器编号（Mid-360 有 4 个）
  uint32_t offset_time = 0;  // 相对帧首的纳秒偏移
};

class LivoxPreprocess {
 public:
  explicit LivoxPreprocess(const Config& cfg);

  // 转换并过滤。输出点的 curvature 字段存「相对帧首的毫秒偏移」，
  // 这是去畸变的唯一依据，必须准确。
  //
  // point_filter_num：每 N 个点取 1 个。Mid-360 单帧约 2 万点，
  // 本系统用整帧点云，降采样主要靠体素滤波，这里只做粗筛。
  void process(const std::vector<LivoxRawPoint>& raw, double header_time,
               CloudPtr out);

 private:
  // tag 过滤：剔除低置信度回波和雨雾噪点。
  // Livox 官方定义：tag 的 bit[1:0] == 0 表示非噪点，bit[3:2] == 0 表示非粗噪点。
  bool isTagValid(uint8_t tag) const;

  Config cfg_;
  int    point_filter_num_ = 1;
};

}  // namespace lvi
