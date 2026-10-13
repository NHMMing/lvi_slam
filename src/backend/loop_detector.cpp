// lvi_slam/backend/loop_detector.cpp
// 双模态回环检测：Scan Context（激光）+ ORB 描述子（视觉），两者互相验证。
//
// 为什么不用 LIO-SAM 的半径搜索：它依赖里程计精度 —— 漂移超过搜索半径就检测不到，
// 这是个死循环（越需要回环时越检测不到）。Scan Context 是场景描述子，不依赖位置先验。
//
// 为什么要双模态：误回环是因子图的头号杀手。两个模态都认同才接受，
// 能把误检率压下一个数量级。代价只是多存一份描述子。
#include "lvi_slam/backend/loop_detector.h"

#include <pcl/registration/icp.h>
#include <algorithm>

namespace lvi {
namespace {
constexpr float kNoPoint = -1000.0f;
}

LoopDetector::LoopDetector(const Config& cfg) : cfg_(cfg) {
  // ORB 是二进制描述子，用汉明距离
  matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
}

// ---------------------------------------------------------------------------
// Scan Context：ring × sector 网格，每个 bin 存落入其中点的最大高度。
//
// 存最大高度而不是点数或平均高度，是因为它对地面点不敏感，突出结构物 ——
// 地面在所有位置都一样，对区分场景没有贡献。
// ---------------------------------------------------------------------------
Eigen::MatrixXf LoopDetector::makeScanContext(const CloudPtr& cloud) const {
  Eigen::MatrixXf sc = Eigen::MatrixXf::Constant(num_ring_, num_sector_, kNoPoint);
  if (!cloud) return sc;

  for (const auto& p : cloud->points) {
    const float r = std::sqrt(p.x * p.x + p.y * p.y);
    if (r > max_radius_ || r < 1e-3f) continue;

    float theta = std::atan2(p.y, p.x) * 180.0f / static_cast<float>(M_PI);
    if (theta < 0) theta += 360.0f;

    const int ring = std::min(num_ring_ - 1,
                              static_cast<int>(r / max_radius_ * num_ring_));
    const int sec = std::min(num_sector_ - 1,
                             static_cast<int>(theta / 360.0f * num_sector_));
    if (p.z > sc(ring, sec)) sc(ring, sec) = p.z;
  }
  return sc;
}

// Ring key：对每一行（ring）在 sector 维度求均值。
// 列移位不改变行均值，所以它是旋转不变的 —— 用来做粗筛，避免对每个候选都跑完整的移位搜索。
Eigen::VectorXf LoopDetector::makeRingKey(const Eigen::MatrixXf& sc) const {
  Eigen::VectorXf key = Eigen::VectorXf::Zero(sc.rows());
  for (int i = 0; i < sc.rows(); ++i) {
    float sum = 0.f;
    int cnt = 0;
    for (int j = 0; j < sc.cols(); ++j) {
      if (sc(i, j) > kNoPoint) { sum += sc(i, j); ++cnt; }
    }
    key(i) = (cnt > 0) ? sum / static_cast<float>(cnt) : 0.f;
  }
  return key;
}

// 列向量间的余弦距离，对所有循环移位取最小。
// 移位量本身有物理含义 —— 它就是两帧之间的偏航角，后面直接用作 ICP 初值。
double LoopDetector::scDistance(const Eigen::MatrixXf& a, const Eigen::MatrixXf& b,
                                int* best_shift) const {
  double min_dist = 1.0;
  int min_shift = 0;
  const int n_sec = static_cast<int>(a.cols());

  for (int shift = 0; shift < n_sec; ++shift) {
    double sum_cos = 0.0;
    int valid = 0;
    for (int j = 0; j < n_sec; ++j) {
      const Eigen::VectorXf ca = a.col(j);
      const Eigen::VectorXf cb = b.col((j + shift) % n_sec);
      const double na = ca.norm(), nb = cb.norm();
      if (na < 1e-6 || nb < 1e-6) continue;
      sum_cos += ca.dot(cb) / (na * nb);
      ++valid;
    }
    if (valid == 0) continue;
    const double dist = 1.0 - sum_cos / valid;
    if (dist < min_dist) { min_dist = dist; min_shift = shift; }
  }
  if (best_shift) *best_shift = min_shift;
  return min_dist;
}

void LoopDetector::addKeyframe(const KeyframePtr& kf) {
  if (!kf || !kf->is_loop_candidate) return;   // 前端已按空间间隔稀疏化
  if (!kf->cloud_body || kf->cloud_body->empty()) return;

  const Eigen::MatrixXf sc = makeScanContext(kf->cloud_body);
  kf->scan_context = sc;

  sc_database_.push_back(sc);
  ringkey_database_.push_back(makeRingKey(sc));
  kf_id_database_.push_back(kf->id);
}

bool LoopDetector::detect(const KeyframePtr& current,
                          const std::vector<KeyframePtr>& all_kfs,
                          LoopConstraint* out) {
  if (!current || !current->cloud_body || sc_database_.size() < 2) return false;
  if (current->id == last_query_id_) return false;      // 同一帧不重复查
  last_query_id_ = current->id;

  const Eigen::MatrixXf sc_cur = makeScanContext(current->cloud_body);
  const Eigen::VectorXf key_cur = makeRingKey(sc_cur);

  // ---- 第一级：Scan Context 检索 ----
  int best_db_idx = -1;
  double best_dist = 1.0;
  int best_shift = 0;

  for (size_t i = 0; i < sc_database_.size(); ++i) {
    const int cand_id = kf_id_database_[i];
    if (cand_id >= static_cast<int>(all_kfs.size())) continue;

    // 时间间隔门槛：排除相邻帧的「假回环」。
    // 相邻帧当然长得像，但它们之间已经有里程计因子了，加回环边没有信息增益。
    if (current->t - all_kfs[cand_id]->t < cfg_.backend.loop_min_time_diff) continue;

    // 粗筛：ring key 距离。这一步把候选从上千个砍到几十个，
    // 避免对每个候选都跑 O(n_sector) 的移位搜索。
    if ((key_cur - ringkey_database_[i]).norm() > 1.0f) continue;

    int shift = 0;
    const double d = scDistance(sc_cur, sc_database_[i], &shift);
    if (d < best_dist) { best_dist = d; best_db_idx = static_cast<int>(i); best_shift = shift; }
  }

  if (best_db_idx < 0 || best_dist > cfg_.backend.sc_dist_thresh) return false;

  const int prev_id = kf_id_database_[best_db_idx];
  if (prev_id < 0 || prev_id >= static_cast<int>(all_kfs.size())) return false;
  const KeyframePtr& prev = all_kfs[prev_id];

  out->id_from = current->id;
  out->id_to = prev_id;
  out->sc_hit = true;
  out->sc_dist = best_dist;

  // 把 Scan Context 的移位量转成偏航角，作为 ICP 初值。
  // 没有这个初值，ICP 在大偏航角下基本必然收敛到局部极小。
  sc_yaw_init_ = -2.0 * M_PI * best_shift / num_sector_;

  // ---- 第二级：ICP 几何验证 ----
  if (!verifyByIcp(current, prev, all_kfs, out)) return false;
  out->verified_lidar = true;

  // ---- 第三级：视觉交叉验证 ----
  if (cfg_.backend.dual_modal_verify) {
    out->verified_visual = verifyByVisual(current, prev);
    // 两帧都有描述子却匹配不上 → 大概率是几何巧合（走廊、对称结构），拒绝
    if (!current->descriptors.empty() && !prev->descriptors.empty() &&
        !out->verified_visual) {
      return false;
    }
  }
  return true;
}

// ICP 验证。当前帧对历史帧前后 N 帧拼成的子图 ——
// 单帧点云太稀疏，重叠率不够，ICP 配不准。
bool LoopDetector::verifyByIcp(const KeyframePtr& cur, const KeyframePtr& prev,
                               const std::vector<KeyframePtr>& all_kfs,
                               LoopConstraint* out) {
  CloudPtr src(new PointCloud()), tgt(new PointCloud());

  auto accumulate = [&](int center_id, int half_span, CloudPtr dst) {
    const int total = static_cast<int>(all_kfs.size());
    for (int i = -half_span; i <= half_span; ++i) {
      const int idx = center_id + i;
      if (idx < 0 || idx >= total) continue;
      const auto& kf = all_kfs[idx];
      if (!kf || !kf->cloud_body) continue;
      for (const auto& p : kf->cloud_body->points) {
        const V3D pw = kf->rot * V3D(p.x, p.y, p.z) + kf->pos;
        PointType q = p;
        q.x = static_cast<float>(pw.x());
        q.y = static_cast<float>(pw.y());
        q.z = static_cast<float>(pw.z());
        dst->push_back(q);
      }
    }
  };

  accumulate(cur->id, 0, src);
  accumulate(prev->id, 25, tgt);
  if (src->size() < 300 || tgt->size() < 1000) return false;

  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(30.0);
  icp.setMaximumIterations(cfg_.backend.icp_max_iter);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  guess.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(static_cast<float>(sc_yaw_init_),
                        Eigen::Vector3f::UnitZ()).toRotationMatrix();

  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  PointCloud aligned;
  icp.align(aligned, guess);

  if (!icp.hasConverged()) return false;
  const double fitness = icp.getFitnessScore();
  out->icp_fitness = fitness;
  if (fitness > cfg_.backend.icp_fitness_thresh) return false;

  // ICP 给的是「把当前帧点云对齐到历史子图」的修正量。
  // 转成因子图需要的 T_prev_cur（历史帧坐标系下看当前帧）。
  const Eigen::Matrix4f T = icp.getFinalTransformation();
  const M3D dR = T.block<3, 3>(0, 0).cast<double>();
  const V3D dt = T.block<3, 1>(0, 3).cast<double>();

  const M3D rot_cur_fixed = dR * cur->rot;
  const V3D pos_cur_fixed = dR * cur->pos + dt;

  out->rot_delta = prev->rot.transpose() * rot_cur_fixed;
  out->pos_delta = prev->rot.transpose() * (pos_cur_fixed - prev->pos);
  // 噪声用 fitness 自适应：配得越好权重越大。简洁且有效。
  out->noise = std::max(fitness, 1e-4);
  return true;
}

// 视觉验证：ORB 描述子互匹配 + Lowe ratio test。
// 只看内点数量，不求位姿 —— 位姿已经由 ICP 给了，这里只回答
// 「这两帧看到的是不是同一个地方」。
bool LoopDetector::verifyByVisual(const KeyframePtr& cur, const KeyframePtr& prev) {
  if (cur->descriptors.empty() || prev->descriptors.empty()) return false;

  std::vector<std::vector<cv::DMatch>> knn;
  matcher_->knnMatch(cur->descriptors, prev->descriptors, knn, 2);

  int good = 0;
  for (const auto& m : knn) {
    if (m.size() < 2) continue;
    // Lowe ratio test：最近邻明显优于次近邻才算可靠匹配
    if (m[0].distance < 0.75f * m[1].distance) ++good;
  }
  return good >= 25;
}

}  // namespace lvi
