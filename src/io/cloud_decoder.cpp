#include "lvi_slam/io/cloud_decoder.h"
#include <cstring>
#include <iostream>
#include <algorithm>

namespace lvi {
namespace {
// 对应 sensor_msgs::msg::PointField 的类型常量
constexpr uint8_t INT8 = 1, UINT8 = 2, INT16 = 3, UINT16 = 4,
                  INT32 = 5, UINT32 = 6, FLOAT32 = 7, FLOAT64 = 8;
}

double FieldAccessor::read(const uint8_t* base) const {
  const uint8_t* p = base + offset;
  switch (datatype) {
    case INT8:    { int8_t   v; std::memcpy(&v, p, 1); return v; }
    case UINT8:   { uint8_t  v; std::memcpy(&v, p, 1); return v; }
    case INT16:   { int16_t  v; std::memcpy(&v, p, 2); return v; }
    case UINT16:  { uint16_t v; std::memcpy(&v, p, 2); return v; }
    case INT32:   { int32_t  v; std::memcpy(&v, p, 4); return v; }
    case UINT32:  { uint32_t v; std::memcpy(&v, p, 4); return v; }
    case FLOAT32: { float    v; std::memcpy(&v, p, 4); return v; }
    case FLOAT64: { double   v; std::memcpy(&v, p, 8); return v; }
    default: return 0.0;
  }
}

// ---------------------------------------------------------------------------
// 内置剖面
// ---------------------------------------------------------------------------
CloudProfile CloudProfile::livox() {
  CloudProfile p;
  p.name = "livox";
  p.field_time = "offset_time";
  p.field_ring = "line";
  p.time_semantic = TimeSemantic::kRelativeNanos;
  return p;
}

CloudProfile CloudProfile::ouster() {
  CloudProfile p;
  p.name = "ouster";
  p.field_time = "t";                // Ouster 用单字母 t，别和 Velodyne 的 time 搞混
  p.field_ring = "ring";
  p.time_semantic = TimeSemantic::kRelativeNanos;
  return p;
}

CloudProfile CloudProfile::velodyne() {
  CloudProfile p;
  p.name = "velodyne";
  p.field_time = "time";
  p.field_ring = "ring";
  p.time_semantic = TimeSemantic::kRelativeSec;
  return p;
}

CloudProfile CloudProfile::hesai() {
  CloudProfile p;
  p.name = "hesai";
  p.field_time = "timestamp";        // 注意是绝对时间，不是相对偏移
  p.field_ring = "ring";
  p.time_semantic = TimeSemantic::kAbsoluteSec;
  return p;
}

CloudProfile CloudProfile::kitti() {
  CloudProfile p;
  p.name = "kitti";
  p.field_time.clear();
  p.field_ring.clear();
  p.time_semantic = TimeSemantic::kNone;
  p.scan_period = 0.1;
  return p;
}

bool CloudProfile::fromName(const std::string& name, CloudProfile* out) {
  if (name == "livox")         { *out = livox();    return true; }
  if (name == "ouster")        { *out = ouster();   return true; }
  if (name == "velodyne")      { *out = velodyne(); return true; }
  if (name == "hesai")         { *out = hesai();    return true; }
  if (name == "kitti")         { *out = kitti();    return true; }
  return false;
}

// ---------------------------------------------------------------------------
// CloudDecoder
// ---------------------------------------------------------------------------
CloudDecoder::CloudDecoder(const Config& cfg, const CloudProfile& profile)
    : cfg_(cfg), profile_(profile) {}

bool CloudDecoder::bindFields(const RawCloud2& raw) {
  auto find = [&](const std::string& name, FieldAccessor* acc) {
    if (name.empty()) return;
    for (const auto& f : raw.fields) {
      if (f.name == name) { acc->offset = static_cast<int>(f.offset);
                            acc->datatype = f.datatype; return; }
    }
  };
  find(profile_.field_x, &acc_x_);
  find(profile_.field_y, &acc_y_);
  find(profile_.field_z, &acc_z_);
  find(profile_.field_intensity, &acc_i_);
  find(profile_.field_time, &acc_t_);
  find(profile_.field_ring, &acc_ring_);

  if (!acc_x_.valid() || !acc_y_.valid() || !acc_z_.valid()) {
    std::cerr << "[CloudDecoder] 找不到 xyz 字段。实际字段: ";
    for (const auto& f : raw.fields) std::cerr << f.name << " ";
    std::cerr << "\n";
    return false;
  }

  // 声明了时间字段却找不到 —— 这几乎肯定是 profile 选错了，必须报错而不是静默降级。
  // 静默降级会让去畸变失效，而结果只是「精度差一点」，极难排查。
  if (!profile_.field_time.empty() && !acc_t_.valid()) {
    std::cerr << "[CloudDecoder] profile '" << profile_.name << "' 期望时间字段 '"
              << profile_.field_time << "'，但点云里没有。实际字段: ";
    for (const auto& f : raw.fields) std::cerr << f.name << " ";
    std::cerr << "\n  → 检查 dataset profile 是否选对\n";
    return false;
  }

  point_step_ = raw.point_step;
  fields_bound_ = true;
  return true;
}

// 把原始时间值转成「相对帧首的秒」。三种语义在这里收敛成一种。
double CloudDecoder::normalizeTime(double raw_value, double header_time) const {
  switch (profile_.time_semantic) {
    case TimeSemantic::kRelativeSec:   return raw_value;
    case TimeSemantic::kRelativeNanos: return raw_value * 1e-9;
    case TimeSemantic::kAbsoluteSec:   return raw_value - header_time;
    case TimeSemantic::kNone:          return 0.0;   // 调用方按方位角处理
  }
  return 0.0;
}

size_t CloudDecoder::decode(const RawCloud2& raw, CloudPtr out) {
  if (!out) return 0;
  out->clear();
  if (!raw.data || raw.data_size == 0) return 0;

  if (!fields_bound_ && !bindFields(raw)) return 0;
  if (raw.point_step != point_step_) {            // 序列中途换了布局，重新绑定
    fields_bound_ = false;
    if (!bindFields(raw)) return 0;
  }

  const size_t n = static_cast<size_t>(raw.width) * std::max<uint32_t>(raw.height, 1);
  const double blind_min_sq = cfg_.lidar.blind_min * cfg_.lidar.blind_min;
  const double blind_max_sq = cfg_.lidar.blind_max * cfg_.lidar.blind_max;
  const int filter_num = std::max(1, cfg_.lidar.point_filter_num);

  out->reserve(n / filter_num);

  double t_min = 1e18, t_max = -1e18;

  for (size_t i = 0; i < n; ++i) {
    if (i % filter_num != 0) continue;
    const uint8_t* base = raw.data + i * raw.point_step;

    const double x = acc_x_.read(base);
    const double y = acc_y_.read(base);
    const double z = acc_z_.read(base);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    const double d2 = x * x + y * y + z * z;
    if (d2 < blind_min_sq || d2 > blind_max_sq) continue;

    double rel_sec = 0.0;
    if (profile_.time_semantic == TimeSemantic::kNone) {
      // 无逐点时间：按方位角反推。
      // 这是有损的近似 —— 假设匀速旋转且一圈正好 scan_period。
      // KITTI 这类老数据集只能这么办，精度天然不如有时间戳的。
      double azimuth = std::atan2(y, x);
      if (azimuth < 0) azimuth += 2.0 * M_PI;
      const double ratio = profile_.clockwise ? (1.0 - azimuth / (2.0 * M_PI))
                                              : (azimuth / (2.0 * M_PI));
      rel_sec = ratio * profile_.scan_period;
    } else if (acc_t_.valid()) {
      rel_sec = normalizeTime(acc_t_.read(base), raw.header_time);
    }

    t_min = std::min(t_min, rel_sec);
    t_max = std::max(t_max, rel_sec);

    PointType p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(z);
    p.intensity = acc_i_.valid() ? static_cast<float>(acc_i_.read(base)) : 0.f;
    p.curvature = static_cast<float>(rel_sec * 1000.0);   // 全系统统一：毫秒
    out->push_back(p);
  }

  // ---- 时间合理性自检 ----
  // 这是最有价值的一段代码。时间语义配错时，这里会立刻报警，
  // 而不是让你在三天后对着一个「精度就是上不去」的轨迹发愁。
  if (!warned_time_range_ && !out->empty() &&
      profile_.time_semantic != TimeSemantic::kNone) {
    const double span = t_max - t_min;
    // 一帧点云的时间跨度应该在 [1ms, 500ms]。超出说明语义搞错了量级。
    if (span < 1e-3 || span > 0.5) {
      std::cerr << "[CloudDecoder] 警告：单帧时间跨度 " << span << " 秒，不合常理。\n"
                << "  profile='" << profile_.name << "' 时间语义="
                << static_cast<int>(profile_.time_semantic) << "\n"
                << "  → 很可能是相对/绝对、秒/纳秒 搞混了。去畸变会失效。\n";
      warned_time_range_ = true;
    }
  }

  // 有些数据集的 t_min 不是 0（第一个点不在帧首）。统一平移到 0 为基准。
  if (!out->empty() && std::fabs(t_min) > 1e-9 &&
      profile_.time_semantic != TimeSemantic::kNone) {
    const float shift = static_cast<float>(t_min * 1000.0);
    for (auto& p : out->points) p.curvature -= shift;
  }

  return out->size();
}

}  // namespace lvi
