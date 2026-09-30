#include "lvi_slam/preprocess/livox_preprocess.h"

namespace lvi {

LivoxPreprocess::LivoxPreprocess(const Config& cfg)
    : cfg_(cfg), point_filter_num_(cfg.lidar.point_filter_num) {}

bool LivoxPreprocess::isTagValid(uint8_t tag) const {
  if (!cfg_.lidar.use_livox_tag) return true;
  // bit[1:0]: 雨雾噪点等级，bit[3:2]: 拖点/粗噪点等级。
  // 两者都为 0 才算干净点。这是 Livox 硬件层给的先验，比后续任何滤波都准。
  const uint8_t atmos = tag & 0x03;
  const uint8_t drag  = (tag >> 2) & 0x03;
  return atmos == 0 && drag == 0;
}

void LivoxPreprocess::process(const std::vector<LivoxRawPoint>& raw,
                              double header_time, CloudPtr out) {
  (void)header_time;   // 时间基准由 SyncPackage 统一管理，这里只填相对偏移
  if (!out) return;
  out->clear();
  out->reserve(raw.size() / std::max(1, point_filter_num_));

  const double blind_min_sq = cfg_.lidar.blind_min * cfg_.lidar.blind_min;
  const double blind_max_sq = cfg_.lidar.blind_max * cfg_.lidar.blind_max;

  for (size_t i = 0; i < raw.size(); ++i) {
    if (static_cast<int>(i) % point_filter_num_ != 0) continue;

    const auto& p = raw[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (!isTagValid(p.tag)) continue;

    const double d2 = static_cast<double>(p.x) * p.x +
                      static_cast<double>(p.y) * p.y +
                      static_cast<double>(p.z) * p.z;
    if (d2 < blind_min_sq || d2 > blind_max_sq) continue;

    PointType q;
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    q.intensity = p.reflectivity;
    // 纳秒 → 毫秒。curvature 字段被复用为时间偏移，全系统统一约定。
    q.curvature = static_cast<float>(p.offset_time) * 1e-6f;
    out->push_back(q);
  }
}

}  // namespace lvi
