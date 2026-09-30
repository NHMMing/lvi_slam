// lvi_slam/io/cloud_decoder.h
// 点云解码：把各家不同的 PointCloud2 字段布局归一化成内部格式。
//
// 这是整个加载器最容易出错的地方，也是必须显式配置、不能自动猜的地方。
//
// 各数据集的字段布局与时间语义：
//
//   数据集        LiDAR          时间字段        类型      语义
//   ---------------------------------------------------------------------
//   MARS-LVIG    Livox Avia     offset_time    uint32    相对帧首，纳秒
//   R3LIVE       Livox Avia     offset_time    uint32    相对帧首，纳秒
//   NTU VIRAL    Ouster OS1     t              uint32    相对帧首，纳秒
//   Newer Coll.  Ouster OS1     t              uint32    相对帧首，纳秒
//   Hilti'22/23  Hesai XT-32    timestamp      float64   绝对 UNIX 时间（秒）
//   KITTI        Velodyne       (无)           —         需按方位角反推
//   通用 Velodyne Velodyne      time           float32   相对帧首，秒
//
// 内部统一约定：PointType.curvature 存「相对帧首的毫秒偏移」。
// 这个约定全系统只在这里和 Livox 预处理器里落地，其他地方一律信任它。
#pragma once

#include "lvi_slam/common/types.h"
#include "lvi_slam/common/config.h"
#include <string>
#include <vector>

namespace lvi {

// 时间字段的语义。选错不会崩溃，只会让去畸变系统性偏差 —— 必须显式配置。
enum class TimeSemantic {
  kRelativeSec,    // float，相对帧首，秒（Velodyne 惯例）
  kRelativeNanos,  // uint32，相对帧首，纳秒（Ouster / Livox 惯例）
  kAbsoluteSec,    // double，绝对 UNIX 时间，秒（Hesai 惯例）
  kNone,           // 无逐点时间，按方位角反推（KITTI）
};

// 一个「解码剖面」描述某个数据集的点云长什么样。
// 新增数据集 = 加一个 profile，不改任何代码。
struct CloudProfile {
  std::string name = "generic";

  // 字段名。留空表示该字段不存在。
  std::string field_x = "x";
  std::string field_y = "y";
  std::string field_z = "z";
  std::string field_intensity = "intensity";
  std::string field_time = "time";
  std::string field_ring = "ring";

  TimeSemantic time_semantic = TimeSemantic::kRelativeSec;

  // kNone 时用：一圈的时长，按方位角线性插值出每点时间
  double scan_period = 0.1;
  bool   clockwise = true;      // 扫描方向，影响方位角到时间的映射

  // 内置剖面
  static CloudProfile livox();      // MARS-LVIG / R3LIVE / 自采
  static CloudProfile ouster();     // NTU VIRAL / Newer College
  static CloudProfile velodyne();   // 通用 Velodyne
  static CloudProfile hesai();      // Hilti'22 / '23
  static CloudProfile kitti();      // 无逐点时间
  static bool fromName(const std::string& name, CloudProfile* out);
};

// 一个字段在原始 buffer 里的位置和类型。解析一次，复用整个序列。
struct FieldAccessor {
  int      offset = -1;
  uint8_t  datatype = 0;   // 对应 sensor_msgs::PointField 的类型枚举
  bool     valid() const { return offset >= 0; }

  // 按 datatype 从 buffer 里取值并转成 double。
  // 不做类型假设 —— 同一个字段名在不同数据集里可能是 float32 也可能是 uint32。
  double read(const uint8_t* base) const;
};

// PointCloud2 的最小抽象。核心库不依赖 ROS 消息类型，
// ROS 层/bag 读取层负责填这个结构。
struct RawCloud2 {
  double header_time = 0.0;
  uint32_t width = 0, height = 0;
  uint32_t point_step = 0, row_step = 0;
  bool is_dense = true;
  struct Field { std::string name; uint32_t offset; uint8_t datatype; uint32_t count; };
  std::vector<Field> fields;
  const uint8_t* data = nullptr;
  size_t data_size = 0;
};

class CloudDecoder {
 public:
  explicit CloudDecoder(const Config& cfg, const CloudProfile& profile);

  // 解码 + 过滤 + 时间归一化。输出 curvature = 相对帧首毫秒。
  // 返回实际输出点数；0 表示本帧无效。
  size_t decode(const RawCloud2& raw, CloudPtr out);

  const CloudProfile& profile() const { return profile_; }

 private:
  // 首次调用时解析字段偏移，之后复用（同一序列字段布局不变）
  bool bindFields(const RawCloud2& raw);
  // 把原始时间值转成「相对帧首的秒」
  double normalizeTime(double raw_value, double header_time) const;

  Config       cfg_;
  CloudProfile profile_;

  FieldAccessor acc_x_, acc_y_, acc_z_, acc_i_, acc_t_, acc_ring_;
  bool   fields_bound_ = false;
  size_t point_step_ = 0;

  // 诊断：时间字段异常时只警告一次，不刷屏
  bool   warned_time_range_ = false;
};

}  // namespace lvi
