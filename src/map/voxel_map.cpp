// lvi_slam/map/voxel_map.cpp
// 统一体素地图：几何平面与视觉点共用同一套哈希索引。
//
// 这是整个系统耦合最深的地方。给一个视觉点，能 O(1) 查到它所在体素的
// 平面法向量和法向量的不确定度，从而：
//   1. 用精确的平面单应 H = R - t·nᵀ/d 做 patch 仿射变换（不用 fronto-parallel 假设）
//   2. 用平面协方差对法向量更新做卡方检验
// 这是 LVI-SAM 那种「两个子系统互相提供初值」的架构做不到的。
#include "lvi_slam/map/voxel_map.h"

#include <Eigen/Eigenvalues>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace lvi {

// ---------------------------------------------------------------------------
// 逐点噪声模型：整个权重体系的物理起点。
//
// LiDAR 点的不确定度不是各向同性的：
//   沿光束方向   —— 测距误差，固定（约 2cm）
//   垂直光束方向 —— 角分辨率误差 × 距离，随距离线性放大
// 结果是一个沿光束方向的细长椭球。远处的点横向极不确定。
//
// 不从这里推导，就只能靠手工调权重（LOAM/LIO-SAM 的做法），换个传感器全部重调。
// ---------------------------------------------------------------------------
void VoxelMap::calcPointCov(const V3D& p, double depth_err, double beam_err_deg,
                            M3D* cov) {
  const double range = p.norm();
  if (range < 1e-6) { *cov = M3D::Identity() * 1e-6; return; }

  const V3D dir = p / range;
  const double beam_rad = beam_err_deg * M_PI / 180.0;
  const double lateral_sigma = range * std::tan(beam_rad);

  // 构造与光束正交的两个基
  V3D base1 = (std::fabs(dir.x()) < 0.9) ? V3D(1, 0, 0) : V3D(0, 1, 0);
  base1 = (base1 - dir * dir.dot(base1)).normalized();
  const V3D base2 = dir.cross(base1);

  Eigen::Matrix<double, 3, 2> N;
  N.col(0) = base1;
  N.col(1) = base2;

  *cov = dir * (depth_err * depth_err) * dir.transpose() +
         N * (Eigen::Matrix2d::Identity() * lateral_sigma * lateral_sigma) * N.transpose();
}

// ---------------------------------------------------------------------------
// VoxelOctoTree
// ---------------------------------------------------------------------------
VoxelOctoTree::VoxelOctoTree(const VoxelMapConfig& cfg, int layer, int max_layer)
    : cfg_(cfg), layer_(layer), max_layer_(max_layer) {}

VoxelOctoTree::~VoxelOctoTree() {
  for (auto*& c : children_) { delete c; c = nullptr; }
}

void VoxelOctoTree::insertPoint(const PointWithVar& pv) {
  if (static_cast<int>(temp_points_.size()) < cfg_.max_points_num * 2) {
    temp_points_.push_back(pv);
  }
}

// PCA 拟合平面 + 计算平面参数自身的协方差。
//
// plane_var 是本系统相对 LOAM/LIO-SAM 最本质的进步。传统方法假设拟合出的平面
// 是精确的；实际上由 6 个点拟合的平面和由 50 个点拟合的平面，可靠性差一个量级。
// 把这个差异量化，权重才有物理意义，卡方检验才有意义。
void VoxelOctoTree::fitPlane() {
  const int n = static_cast<int>(temp_points_.size());
  if (n < 6) { if (plane_) plane_->is_plane = false; return; }

  if (!plane_) plane_ = std::make_unique<VoxelPlane>();

  V3D center = V3D::Zero();
  for (const auto& pv : temp_points_) center += pv.point_w;
  center /= static_cast<double>(n);

  M3D cov = M3D::Zero();
  for (const auto& pv : temp_points_) {
    const V3D d = pv.point_w - center;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(n);

  Eigen::SelfAdjointEigenSolver<M3D> es(cov);
  const V3D evals = es.eigenvalues();     // 升序
  const M3D evecs = es.eigenvectors();

  plane_->center     = center;
  plane_->point_cov  = cov;
  plane_->min_eigen  = evals(0);
  plane_->mid_eigen  = evals(1);
  plane_->max_eigen  = evals(2);
  plane_->points_num = n;
  plane_->normal     = evecs.col(0);      // 最小特征值方向即法向量
  plane_->d          = -plane_->normal.dot(center);
  plane_->radius     = std::sqrt(evals(2));
  plane_->is_plane   = (evals(0) < cfg_.plane_threshold);

  if (!plane_->is_plane) return;

  // ---- 平面参数 (n, q) 的协方差传播 ----
  // 对每个观测点求 ∂[n; q]/∂p_i，再把点协方差传过来。
  // 法向量对点的雅可比由特征值扰动理论给出：法向量的变化由其余两个主方向张成。
  Eigen::Matrix<double, 6, 6> plane_var = Eigen::Matrix<double, 6, 6>::Zero();
  const double inv_n = 1.0 / static_cast<double>(n);

  for (const auto& pv : temp_points_) {
    Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero();
    J.block<3, 3>(3, 0) = M3D::Identity() * inv_n;    // ∂q/∂p_i = I/n

    const V3D d = pv.point_w - center;
    M3D J_n = M3D::Zero();
    for (int k = 1; k < 3; ++k) {
      const double gap = evals(0) - evals(k);
      if (std::fabs(gap) < 1e-9) continue;            // 特征值简并，跳过该方向
      J_n += (evecs.col(k) * (evecs.col(k).transpose() * d) * inv_n / gap)
             * evecs.col(0).transpose();
    }
    J.block<3, 3>(0, 0) = J_n;
    plane_var += J * pv.var * J.transpose();
  }
  plane_->plane_var = plane_var;
}

void VoxelOctoTree::initOctoTree() {
  if (static_cast<int>(temp_points_.size()) < 6) return;
  fitPlane();
  init_done_ = true;

  // 不是平面且还能再分 → 细分。
  // 这就是「自适应」的含义：结构简单的区域用大体素省内存，复杂区域自动细分保精度。
  if ((!plane_ || !plane_->is_plane) && layer_ < max_layer_) split();
}

void VoxelOctoTree::split() {
  is_leaf_ = false;
  const double child_quater = quater_length_ * 0.5;

  for (const auto& pv : temp_points_) {
    const int idx = childIndex(pv.point_w);
    ensureChild(idx, child_quater);
    children_[idx]->insertPoint(pv);
  }
  temp_points_.clear();
  temp_points_.shrink_to_fit();
  for (auto* c : children_) if (c) c->initOctoTree();
}

int VoxelOctoTree::childIndex(const V3D& p) const {
  int idx = 0;
  if (p.x() > center_.x()) idx |= 1;
  if (p.y() > center_.y()) idx |= 2;
  if (p.z() > center_.z()) idx |= 4;
  return idx;
}

void VoxelOctoTree::ensureChild(int idx, double child_quater) {
  if (children_[idx]) return;
  children_[idx] = new VoxelOctoTree(cfg_, layer_ + 1, max_layer_);
  V3D c = center_;
  const double off = quater_length_;
  c.x() += (idx & 1) ? off : -off;
  c.y() += (idx & 2) ? off : -off;
  c.z() += (idx & 4) ? off : -off;
  children_[idx]->setGeometry(c, child_quater);
}

void VoxelOctoTree::updateOctoTree(const PointWithVar& pv) {
  if (!init_done_) {
    insertPoint(pv);
    if (static_cast<int>(temp_points_.size()) >= 6) initOctoTree();
    return;
  }
  if (is_leaf_) {
    // 已成平面的体素：点数没满就继续吸收并重新拟合，平面精度随观测增加而提升，
    // plane_var 也随之减小 —— 权重自然上升，不需要任何手工规则。
    if (static_cast<int>(temp_points_.size()) < cfg_.max_points_num) {
      insertPoint(pv);
      fitPlane();
    }
    return;
  }
  const int idx = childIndex(pv.point_w);
  ensureChild(idx, quater_length_ * 0.5);
  children_[idx]->updateOctoTree(pv);
}

const VoxelOctoTree* VoxelOctoTree::findCorrespond(const V3D& p) const {
  if (is_leaf_ || layer_ >= max_layer_) return this;
  const int idx = childIndex(p);
  if (!children_[idx]) return this;
  return children_[idx]->findCorrespond(p);
}

// ---------------------------------------------------------------------------
// VoxelMap
// ---------------------------------------------------------------------------
VoxelMap::VoxelMap(const Config& cfg) : cfg_(cfg) {}

VoxelMap::~VoxelMap() {
  for (auto& kv : voxels_) delete kv.second;
  voxels_.clear();
}

VoxelOctoTree* VoxelMap::fetchOrCreate(const VoxelKey& key) {
  auto it = voxels_.find(key);
  if (it != voxels_.end()) return it->second;

  const double vs = cfg_.voxel.max_voxel_size;
  auto* node = new VoxelOctoTree(cfg_.voxel, 0, cfg_.voxel.max_layer);
  const V3D center((key.x + 0.5) * vs, (key.y + 0.5) * vs, (key.z + 0.5) * vs);
  node->setGeometry(center, vs * 0.25);
  voxels_[key] = node;
  return node;
}

void VoxelMap::build(const std::vector<PointWithVar>& points) {
  const double vs = cfg_.voxel.max_voxel_size;
  for (const auto& pv : points) {
    fetchOrCreate(VoxelKey::fromPoint(pv.point_w, vs))->insertPoint(pv);
  }
  for (auto& kv : voxels_) kv.second->initOctoTree();
}

void VoxelMap::update(const std::vector<PointWithVar>& points) {
  const double vs = cfg_.voxel.max_voxel_size;
  for (const auto& pv : points) {
    fetchOrCreate(VoxelKey::fromPoint(pv.point_w, vs))->updateOctoTree(pv);
  }
}

// ---------------------------------------------------------------------------
// 点-面残差构建。OpenMP 并行，结果写入预分配数组避免竞争。
//
// 卡方检验同时完成了外点剔除和权重计算 —— 不需要额外的鲁棒核。
// 残差方差 σ² = 平面参数不确定度沿残差方向的投影 + 点噪声沿法向量方向的投影。
// ---------------------------------------------------------------------------
void VoxelMap::buildResidualList(const std::vector<PointWithVar>& points,
                                 std::vector<PointToPlane>* out) const {
  const int n = static_cast<int>(points.size());
  std::vector<PointToPlane> buf(n);
  const double vs = cfg_.voxel.max_voxel_size;
  const double sigma_num = cfg_.voxel.sigma_num;

#ifdef _OPENMP
#pragma omp parallel for num_threads(cfg_.num_threads) schedule(guided)
#endif
  for (int i = 0; i < n; ++i) {
    const auto& pv = points[i];
    auto it = voxels_.find(VoxelKey::fromPoint(pv.point_w, vs));
    if (it == voxels_.end()) continue;

    const VoxelOctoTree* leaf = it->second->findCorrespond(pv.point_w);
    if (!leaf) continue;
    const VoxelPlane* pl = leaf->plane();
    if (!pl || !pl->is_plane) continue;

    const double dist = pl->distanceTo(pv.point_w);

    Eigen::Matrix<double, 1, 6> J_nq;
    J_nq.block<1, 3>(0, 0) = (pv.point_w - pl->center).transpose();
    J_nq.block<1, 3>(0, 3) = -pl->normal.transpose();

    const double sigma_plane = (J_nq * pl->plane_var * J_nq.transpose())(0, 0);
    const double sigma_point = pl->normal.transpose() * pv.var * pl->normal;
    const double sigma_total = sigma_plane + sigma_point;

    if (dist * dist > sigma_num * sigma_num * sigma_total) continue;

    PointToPlane& p = buf[i];
    p.point_b   = pv.point_b;
    p.point_w   = pv.point_w;
    p.normal    = pl->normal;
    p.center    = pl->center;
    p.body_cov  = pv.body_var;
    p.plane_var = pl->plane_var;
    p.dist      = dist;
    p.valid     = true;
  }

  out->clear();
  out->reserve(n);
  for (int i = 0; i < n; ++i) if (buf[i].valid) out->push_back(buf[i]);
}

const VoxelPlane* VoxelMap::queryPlane(const V3D& point_w) const {
  auto it = voxels_.find(VoxelKey::fromPoint(point_w, cfg_.voxel.max_voxel_size));
  if (it == voxels_.end()) return nullptr;
  const VoxelOctoTree* leaf = it->second->findCorrespond(point_w);
  return leaf ? leaf->plane() : nullptr;
}

void VoxelMap::addVisualPoint(const VisualPointPtr& vp) {
  if (!vp) return;
  fetchOrCreate(VoxelKey::fromPoint(vp->pos, cfg_.voxel.max_voxel_size))
      ->visual_points.push_back(vp);
}

void VoxelMap::retrieveVisualPoints(const V3D& center, double radius,
                                    std::vector<VisualPointPtr>* out) const {
  out->clear();
  const double vs = cfg_.voxel.max_voxel_size;
  const int r = static_cast<int>(std::ceil(radius / vs));
  const VoxelKey ck = VoxelKey::fromPoint(center, vs);

  for (int dx = -r; dx <= r; ++dx)
    for (int dy = -r; dy <= r; ++dy)
      for (int dz = -r; dz <= r; ++dz) {
        auto it = voxels_.find(VoxelKey{ck.x + dx, ck.y + dy, ck.z + dz});
        if (it == voxels_.end()) continue;
        for (const auto& vp : it->second->visual_points) {
          if (vp && (vp->pos - center).norm() <= radius) out->push_back(vp);
        }
      }
}

// 局部滑窗。移动超过阈值才触发，避免每帧遍历整张哈希表。
// 纯里程计前端的必然选择 —— 全局地图交给后端的关键帧管理，这里只保局部。
void VoxelMap::slideMap(const V3D& current_pos) {
  if (!cfg_.voxel.sliding_en) return;
  if (!slide_initialized_) {
    last_slide_pos_ = current_pos;
    slide_initialized_ = true;
    return;
  }
  if ((current_pos - last_slide_pos_).norm() < cfg_.voxel.sliding_thresh) return;
  last_slide_pos_ = current_pos;
  clearOutOfRange(VoxelKey::fromPoint(current_pos, cfg_.voxel.max_voxel_size));
}

void VoxelMap::clearOutOfRange(const VoxelKey& c) {
  const int half = cfg_.voxel.half_map_size;
  for (auto it = voxels_.begin(); it != voxels_.end();) {
    const VoxelKey& k = it->first;
    if (std::abs(k.x - c.x) > half || std::abs(k.y - c.y) > half ||
        std::abs(k.z - c.z) > half) {
      delete it->second;
      it = voxels_.erase(it);
    } else {
      ++it;
    }
  }
}

// ---------------------------------------------------------------------------
// 回环后重建局部地图。
//
// 这是前后端同步的落脚点。后端改了关键帧位姿，这里用新位姿把 body 系点云
// 重新变换、重新灌进体素地图。只重建最近 N 帧，代价可控。
//
// 「FAST-LIO2 + 外接位姿图」方案缺的就是这个接口 ——
// 它的 ikd-Tree 里存的是已经变换到世界系的点，位姿改了也改不动。
// ---------------------------------------------------------------------------
void VoxelMap::rebuildFromKeyframes(const std::vector<KeyframePtr>& kfs) {
  for (auto& kv : voxels_) delete kv.second;
  voxels_.clear();
  slide_initialized_ = false;

  std::vector<PointWithVar> pts;
  pts.reserve(kfs.size() * 2000);

  for (const auto& kf : kfs) {
    if (!kf || !kf->cloud_body) continue;
    for (const auto& p : kf->cloud_body->points) {
      PointWithVar pv;
      pv.point_b = V3D(p.x, p.y, p.z);
      pv.point_w = kf->rot * pv.point_b + kf->pos;    // 用修正后的位姿
      calcPointCov(pv.point_b, cfg_.lidar.depth_err, cfg_.lidar.beam_err, &pv.body_var);
      pv.var = kf->rot * pv.body_var * kf->rot.transpose();
      pts.push_back(pv);
    }
  }
  build(pts);
}

}  // namespace lvi
