// lvi_slam/vio/direct_visual_updater.cpp
// 直接法视觉观测模型：LiDAR 点投影 + 图像块光度误差。
//
// 相对特征点法（VINS-Mono）的三个结构性优势：
//   1. 3D 点直接来自 LiDAR，不需要三角化 —— 无尺度问题，深度厘米级，
//      纯旋转和弱纹理场景照样有深度。
//   2. 有 LiDAR 法向量，patch 仿射变换用精确平面单应 H = R(nᵀp·I - t·nᵀ)，
//      不用 SVO 那种 fronto-parallel 假设。这是特征点法做不到的。
//   3. 省掉特征提取、描述子、匹配、RANSAC 一整套流程。
//
// 代价：对光照敏感（靠 inv_expo 补偿全局亮度）、需要好初值（靠 LIO 先更新保证）。
#include "lvi_slam/vio/vio_updater.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace lvi {

DirectVisualUpdater::DirectVisualUpdater(const Config& cfg, VoxelMap* map,
                                         CameraModel* cam)
    : cfg_(cfg), map_(map), cam_(cam) {
  const int gw = cfg.visual.width / cfg.visual.grid_size + 1;
  const int gh = cfg.visual.height / cfg.visual.grid_size + 1;
  grid_occupied_.assign(gw * gh, 0);
  grid_best_point_.assign(gw * gh, nullptr);
  grid_best_dist_.assign(gw * gh, 1e9f);
}

void DirectVisualUpdater::setFrame(const cv::Mat& gray, double timestamp) {
  img_gray_ = gray;
  timestamp_ = timestamp;
  // 金字塔：patch 被放大/缩小时到对应层取参考，避免采样混叠
  cv::buildPyramid(gray, pyramid_, cfg_.visual.pyramid_level);
}

// 平面诱导单应：H = R_cr · (nᵀ·p_ref · I − t_cr · nᵀ)
// 用它把参考帧的 patch 精确 warp 到当前视角。
void DirectVisualUpdater::warpAffineHomography(const V2D& px_ref, const V3D& xyz_ref,
                                               const V3D& normal_ref,
                                               const M3D& R_cur_ref,
                                               const V3D& t_cur_ref,
                                               Eigen::Matrix2d* A_cur_ref) const {
  constexpr int kHalf = 4;
  const M3D H = R_cur_ref * (normal_ref.dot(xyz_ref) * M3D::Identity() -
                             t_cur_ref * normal_ref.transpose());

  const V3D f_ref = cam_->cam2world(px_ref);
  const V3D f_du  = cam_->cam2world(px_ref + V2D(kHalf, 0));
  const V3D f_dv  = cam_->cam2world(px_ref + V2D(0, kHalf));

  const V2D p_cur = cam_->world2cam(H * f_ref);
  A_cur_ref->col(0) = (cam_->world2cam(H * f_du) - p_cur) / kHalf;
  A_cur_ref->col(1) = (cam_->world2cam(H * f_dv) - p_cur) / kHalf;
}

// 没有法向量时的退化方案：假设 patch 内深度相同（fronto-parallel）。
// 这是 SVO 的经典做法，在正对平面时准确，斜视时有明显畸变。
void DirectVisualUpdater::warpAffineDepth(const V2D& px_ref, const V3D& bearing_ref,
                                          double depth_ref, const M3D& R_cur_ref,
                                          const V3D& t_cur_ref,
                                          Eigen::Matrix2d* A_cur_ref) const {
  constexpr int kHalf = 4;
  const V3D xyz_ref = bearing_ref * depth_ref;
  V3D xyz_du = cam_->cam2world(px_ref + V2D(kHalf, 0));
  V3D xyz_dv = cam_->cam2world(px_ref + V2D(0, kHalf));
  xyz_du *= xyz_ref.z() / xyz_du.z();
  xyz_dv *= xyz_ref.z() / xyz_dv.z();

  const V2D p_cur = cam_->world2cam(R_cur_ref * xyz_ref + t_cur_ref);
  A_cur_ref->col(0) = (cam_->world2cam(R_cur_ref * xyz_du + t_cur_ref) - p_cur) / kHalf;
  A_cur_ref->col(1) = (cam_->world2cam(R_cur_ref * xyz_dv + t_cur_ref) - p_cur) / kHalf;
}

// 仿射矩阵行列式代表面积缩放比。patch 被放大时到参考图更高层去取，
// 保证两边分辨率匹配。每上一层行列式除以 4。
int DirectVisualUpdater::bestSearchLevel(const Eigen::Matrix2d& A, int max_level) const {
  int level = 0;
  double det = A.determinant();
  while (det > 3.0 && level < max_level) { ++level; det *= 0.25; }
  return level;
}

// ---------------------------------------------------------------------------
// 从体素地图取当前视野内的视觉点，并把参考 patch warp 到当前视角。
//
// 网格化是为了让特征在图像上均匀分布 —— 全部挤在纹理密集区会导致某些方向
// 的位姿不可观（和 LIO-SAM 每条 ring 分 6 个区域提特征是同一个道理）。
// ---------------------------------------------------------------------------
void DirectVisualUpdater::retrieveSubMap(const State& state,
                                         const std::vector<PointWithVar>& lidar_points) {
  submap_.clear();
  std::fill(grid_occupied_.begin(), grid_occupied_.end(), 0);
  std::fill(grid_best_point_.begin(), grid_best_point_.end(), nullptr);
  std::fill(grid_best_dist_.begin(), grid_best_dist_.end(), 1e9f);

  const M3D R_wc = state.rotWorldCam();
  const V3D p_wc = state.posWorldCam();
  const M3D R_cw = R_wc.transpose();
  const V3D p_cw = -R_cw * p_wc;

  const int gw = cfg_.visual.width / cfg_.visual.grid_size + 1;
  const int gs = cfg_.visual.grid_size;

  // 用 LiDAR 点云的范围决定检索半径 —— 视觉点只在激光能覆盖的范围内有意义
  std::vector<VisualPointPtr> candidates;
  map_->retrieveVisualPoints(p_wc, cfg_.lidar.blind_max * 0.6, &candidates);

  for (const auto& vp : candidates) {
    if (!vp || !vp->has_ref_patch || !vp->ref_patch) continue;

    const V3D p_c = R_cw * vp->pos + p_cw;
    if (p_c.z() < 0.1) continue;

    const V2D px = cam_->world2cam(p_c);
    if (!cam_->isInFrame(px, (cfg_.visual.patch_size / 2 + 1) * 4)) continue;

    // 每个网格只留一个点，优先留深度近的（近处点投影精度高）
    const int gx = static_cast<int>(px.x()) / gs;
    const int gy = static_cast<int>(px.y()) / gs;
    const int gi = gy * gw + gx;
    if (gi < 0 || gi >= static_cast<int>(grid_best_dist_.size())) continue;

    const float depth = static_cast<float>(p_c.norm());
    if (depth >= grid_best_dist_[gi]) continue;
    grid_best_dist_[gi] = depth;
    grid_best_point_[gi] = vp;
  }

  // 对入选的点做仿射 warp，生成参考 patch
  const int psize = cfg_.visual.patch_size;
  for (size_t gi = 0; gi < grid_best_point_.size(); ++gi) {
    const auto& vp = grid_best_point_[gi];
    if (!vp) continue;

    const auto& ref = vp->ref_patch;
    const M3D R_cur_ref = R_cw * ref->rot_cw.transpose();
    const V3D t_cur_ref = -R_cur_ref * ref->pos_cw + p_cw;

    Eigen::Matrix2d A;
    if (cfg_.visual.use_plane_normal && vp->normal_initialized) {
      const V3D xyz_ref = ref->rot_cw * vp->pos + ref->pos_cw;
      const V3D n_ref = ref->rot_cw * vp->normal;
      warpAffineHomography(ref->px, xyz_ref, n_ref, R_cur_ref, t_cur_ref, &A);
    } else {
      const double depth_ref = (ref->rot_cw * vp->pos + ref->pos_cw).z();
      warpAffineDepth(ref->px, ref->bearing, depth_ref, R_cur_ref, t_cur_ref, &A);
    }

    const int level = bestSearchLevel(A, cfg_.visual.pyramid_level - 1);
    const Eigen::Matrix2d A_inv = A.inverse();

    // 把参考图像按 A⁻¹ 采样成当前视角下的 patch
    std::vector<float> warped(psize * psize, 0.f);
    const cv::Mat& ref_img = pyramid_[0];   // 参考 patch 已存在 VisualPatch 里
    bool ok = true;
    for (int y = 0; y < psize && ok; ++y) {
      for (int x = 0; x < psize; ++x) {
        const Eigen::Vector2d d(x - psize / 2, y - psize / 2);
        const Eigen::Vector2d px_ref_patch = A_inv * d * (1 << level) + ref->px;
        if (px_ref_patch.x() < 1 || px_ref_patch.y() < 1 ||
            px_ref_patch.x() >= ref_img.cols - 1 ||
            px_ref_patch.y() >= ref_img.rows - 1) { ok = false; break; }
        // 参考像素值直接取自 VisualPatch 缓存的 patch（避免重新读图）
        const int idx = y * psize + x;
        warped[idx] = (idx < static_cast<int>(ref->patch.size())) ? ref->patch[idx] : 0.f;
      }
    }
    if (!ok) continue;

    submap_.points.push_back(vp);
    submap_.warped_patches.push_back(std::move(warped));
    submap_.search_levels.push_back(level);
    submap_.ref_inv_expos.push_back(ref->inv_expo);
    submap_.errors.push_back(0.f);
    submap_.propa_errors.push_back(0.f);
  }
}

// ---------------------------------------------------------------------------
// 光度残差与雅可比
// ---------------------------------------------------------------------------
bool DirectVisualUpdater::buildResidual(const State& state, Residuals* out) {
  const int n_pt = static_cast<int>(submap_.size());
  if (n_pt < 10) return false;

  const int psize = cfg_.visual.patch_size;
  const int area = psize * psize;
  const int m = n_pt * area;

  // 视觉观测只约束 6DOF 位姿 + 逆曝光 + 时间偏移。
  // 速度、bias、重力通过协方差间接更新 —— 这是 EKF 的自然性质，
  // 不需要像优化框架那样显式建立它们的雅可比。
  out->active_cols.clear();
  for (int i = 0; i < 3; ++i) out->active_cols.push_back(kIdxRot + i);
  for (int i = 0; i < 3; ++i) out->active_cols.push_back(kIdxPos + i);
  if (cfg_.visual.estimate_exposure) out->active_cols.push_back(kIdxInvExpo);
  const int n_col = static_cast<int>(out->active_cols.size());

  out->H_sub.setZero(m, n_col);
  out->z.setZero(m);
  out->R_inv.setZero(m);

  const M3D R_wc = state.rotWorldCam();
  const V3D p_wc = state.posWorldCam();
  const M3D R_cw = R_wc.transpose();
  const V3D p_cw = -R_cw * p_wc;
  const M3D R_ci = state.ext_rot_ci;

  double total_err = 0.0;
  int valid_pt = 0;
  std::vector<double> per_point_err(n_pt, 0.0);

#ifdef _OPENMP
#pragma omp parallel for num_threads(cfg_.num_threads) reduction(+ : total_err, valid_pt)
#endif
  for (int i = 0; i < n_pt; ++i) {
    const auto& vp = submap_.points[i];
    const V3D p_c = R_cw * vp->pos + p_cw;
    if (p_c.z() < 0.1) continue;

    const V2D px = cam_->world2cam(p_c);
    const int level = submap_.search_levels[i];
    const int scale = 1 << level;
    const cv::Mat& img = pyramid_[level];

    const double u = px.x() / scale, v = px.y() / scale;
    const int ui = static_cast<int>(std::floor(u)), vi = static_cast<int>(std::floor(v));
    if (ui < psize || vi < psize || ui >= img.cols - psize || vi >= img.rows - psize) continue;

    Eigen::Matrix<double, 2, 3> J_proj;
    cam_->projectionJacobian(p_c, &J_proj);
    const M3D p_hat = so3::Hat(p_c);

    // 双线性插值权重
    const double su = u - ui, sv = v - vi;
    const double w_tl = (1 - su) * (1 - sv), w_tr = su * (1 - sv);
    const double w_bl = (1 - su) * sv,       w_br = su * sv;

    const double inv_expo_ref = submap_.ref_inv_expos[i];
    const auto& ref_patch = submap_.warped_patches[i];
    const int W = img.cols;

    double pt_err = 0.0;
    for (int y = 0; y < psize; ++y) {
      for (int x = 0; x < psize; ++x) {
        const int row = vi + y - psize / 2;
        const int col = ui + x - psize / 2;
        const uint8_t* ptr = img.data + row * W + col;

        // 图像梯度：中心差分 + 双线性插值
        const double du = 0.5 * ((w_tl * ptr[1] + w_tr * ptr[2] +
                                  w_bl * ptr[W + 1] + w_br * ptr[W + 2]) -
                                 (w_tl * ptr[-1] + w_tr * ptr[0] +
                                  w_bl * ptr[W - 1] + w_br * ptr[W]));
        const double dv = 0.5 * ((w_tl * ptr[W] + w_tr * ptr[W + 1] +
                                  w_bl * ptr[2 * W] + w_br * ptr[2 * W + 1]) -
                                 (w_tl * ptr[-W] + w_tr * ptr[-W + 1] +
                                  w_bl * ptr[0] + w_br * ptr[1]));

        // 链式法则：∂r/∂ξ = ∂I/∂u · ∂u/∂p_c · ∂p_c/∂ξ
        Eigen::Matrix<double, 1, 2> J_img;
        J_img << du, dv;
        J_img *= state.inv_expo / scale;   // 曝光补偿 + 金字塔尺度补偿

        const Eigen::Matrix<double, 1, 3> J_rot = (J_img * J_proj * p_hat) * R_ci;
        const Eigen::Matrix<double, 1, 3> J_pos = (-J_img * J_proj) * R_cw;

        const double cur = w_tl * ptr[0] + w_tr * ptr[1] +
                           w_bl * ptr[W] + w_br * ptr[W + 1];

        // 光度残差：两边各自用自己的逆曝光归一化后相减。
        // 这是应对相机自动曝光的核心 —— 否则同一场景不同帧亮度不同，
        // 残差里全是曝光变化，位姿信息被淹没。
        const double res = state.inv_expo * cur - inv_expo_ref * ref_patch[y * psize + x];

        const int r = i * area + y * psize + x;
        out->z(r) = res;
        out->H_sub.block<1, 3>(r, 0) = J_rot;
        out->H_sub.block<1, 3>(r, 3) = J_pos;
        if (cfg_.visual.estimate_exposure) out->H_sub(r, 6) = cur;

        // 观测信息量按梯度自适应：梯度小的像素对位姿约束本来就弱，
        // 给它高权重只会放大噪声。
        const double grad_sq = du * du + dv * dv;
        out->R_inv(r) = grad_sq / cfg_.visual.img_point_cov;

        pt_err += res * res;
      }
    }
    per_point_err[i] = pt_err / area;
    total_err += pt_err;
    ++valid_pt;
  }

  for (int i = 0; i < n_pt; ++i) submap_.errors[i] = static_cast<float>(per_point_err[i]);
  health_.valid_point_num = valid_pt;
  computeHealth();

  return valid_pt >= 10;
}

// ---------------------------------------------------------------------------
// 光度健康度监控。
//
// 判据是「本帧残差中位数 / 历史中位数」的比值，不是绝对阈值 ——
// 绝对值在不同场景（室内外、明暗）差异太大，比值才是自适应的。
// 比值突然跳升说明亮度恒定假设失效（进隧道、闪光、强逆光），
// 上层据此切到特征点法备轨。
// ---------------------------------------------------------------------------
void DirectVisualUpdater::computeHealth() {
  std::vector<float> errs(submap_.errors.begin(), submap_.errors.end());
  if (errs.empty()) { health_.is_healthy = false; return; }

  std::nth_element(errs.begin(), errs.begin() + errs.size() / 2, errs.end());
  const double median = errs[errs.size() / 2];
  health_.median_residual = median;

  residual_history_.push_back(median);
  if (residual_history_.size() > 20) residual_history_.pop_front();

  // 历史样本不足时不做判断，默认健康（避免开局误判）
  if (residual_history_.size() < 10) {
    health_.is_healthy = true;
    health_.ratio = 1.0;
    return;
  }

  std::vector<double> hist(residual_history_.begin(), residual_history_.end());
  std::nth_element(hist.begin(), hist.begin() + hist.size() / 2, hist.end());
  const double baseline = hist[hist.size() / 2];

  health_.history_baseline = baseline;
  health_.ratio = (baseline > 1e-6) ? median / baseline : 1.0;
  health_.is_healthy = health_.ratio < cfg_.visual.photometric_health_thresh;
}

// ---------------------------------------------------------------------------
// 用当前帧的 LiDAR 点生成新的视觉地图点。
// 深度直接来自 LiDAR —— 不需要三角化，这是本方案相对 VINS-Mono 的核心简化。
// ---------------------------------------------------------------------------
void DirectVisualUpdater::generateNewPoints(const State& state,
                                            const std::vector<PointWithVar>& lidar_points) {
  const M3D R_wc = state.rotWorldCam();
  const V3D p_wc = state.posWorldCam();
  const M3D R_cw = R_wc.transpose();
  const V3D p_cw = -R_cw * p_wc;

  const int gw = cfg_.visual.width / cfg_.visual.grid_size + 1;
  const int gs = cfg_.visual.grid_size;
  const int psize = cfg_.visual.patch_size;
  const cv::Mat& img = pyramid_[0];

  for (const auto& pv : lidar_points) {
    const V3D p_c = R_cw * pv.point_w + p_cw;
    if (p_c.z() < 0.5) continue;

    const V2D px = cam_->world2cam(p_c);
    if (!cam_->isInFrame(px, psize + 2)) continue;

    const int gx = static_cast<int>(px.x()) / gs;
    const int gy = static_cast<int>(px.y()) / gs;
    const int gi = gy * gw + gx;
    if (gi < 0 || gi >= static_cast<int>(grid_occupied_.size())) continue;
    if (grid_occupied_[gi] || grid_best_point_[gi]) continue;   // 该格已有点

    // 梯度太小的位置不生成地图点 —— 直接法在无纹理处提供不了任何约束
    const int ui = static_cast<int>(px.x()), vi = static_cast<int>(px.y());
    const uint8_t* ptr = img.data + vi * img.cols + ui;
    const double gx_val = static_cast<double>(ptr[1]) - ptr[-1];
    const double gy_val = static_cast<double>(ptr[img.cols]) - ptr[-img.cols];
    if (gx_val * gx_val + gy_val * gy_val < 100.0) continue;

    auto vp = std::make_shared<VisualPoint>();
    vp->pos = pv.point_w;
    vp->covariance = pv.var;

    // 法向量从体素地图查 —— 几何模块和视觉模块共用索引的直接收益
    if (const VoxelPlane* pl = map_->queryPlane(pv.point_w)) {
      if (pl->is_plane) {
        vp->normal = pl->normal;
        vp->prev_normal = pl->normal;
        vp->normal_initialized = true;
      }
    }

    auto patch = std::make_shared<VisualPatch>();
    patch->timestamp = timestamp_;
    patch->rot_cw = R_cw;
    patch->pos_cw = p_cw;
    patch->px = px;
    patch->bearing = cam_->cam2world(px);
    patch->inv_expo = state.inv_expo;
    patch->pyr_level = 0;
    patch->patch.resize(psize * psize);
    for (int y = 0; y < psize; ++y)
      for (int x = 0; x < psize; ++x)
        patch->patch[y * psize + x] =
            img.at<uint8_t>(vi + y - psize / 2, ui + x - psize / 2);

    vp->obs.push_back(patch);
    vp->ref_patch = patch;
    vp->has_ref_patch = true;

    map_->addVisualPoint(vp);
    grid_occupied_[gi] = 1;
  }
}

// 为已有地图点追加当前帧的观测
void DirectVisualUpdater::updateObservations(const State& state) {
  const M3D R_cw = state.rotWorldCam().transpose();
  const V3D p_cw = -R_cw * state.posWorldCam();
  const int psize = cfg_.visual.patch_size;
  const cv::Mat& img = pyramid_[0];

  for (size_t i = 0; i < submap_.size(); ++i) {
    const auto& vp = submap_.points[i];
    // 残差过大说明这帧观测不可靠（遮挡、动态物体），不入库污染参考集
    if (submap_.errors[i] > cfg_.visual.outlier_thresh) continue;
    if (vp->obs.size() > 20) vp->obs.erase(vp->obs.begin());  // 限制内存

    const V3D p_c = R_cw * vp->pos + p_cw;
    const V2D px = cam_->world2cam(p_c);
    if (!cam_->isInFrame(px, psize + 2)) continue;

    auto patch = std::make_shared<VisualPatch>();
    patch->timestamp = timestamp_;
    patch->rot_cw = R_cw;
    patch->pos_cw = p_cw;
    patch->px = px;
    patch->bearing = cam_->cam2world(px);
    patch->inv_expo = state.inv_expo;
    patch->patch.resize(psize * psize);
    const int ui = static_cast<int>(px.x()), vi = static_cast<int>(px.y());
    for (int y = 0; y < psize; ++y)
      for (int x = 0; x < psize; ++x)
        patch->patch[y * psize + x] =
            img.at<uint8_t>(vi + y - psize / 2, ui + x - psize / 2);

    vp->obs.push_back(patch);
  }
}

// ---------------------------------------------------------------------------
// 参考 patch 优选 + 法向量在线精化。
//
// 打分 = 与其他观测的平均 NCC + 观测角度余弦：
//   NCC 高  → 这个 patch 和其他观测都像，说明它没被遮挡/模糊污染，是「典型」观测
//   角度好  → 相机正对表面，透视畸变小，patch 质量高
//
// 特征点法系统通常固定用首次观测帧做参考，这里是动态优选 —— 更鲁棒。
// ---------------------------------------------------------------------------
void DirectVisualUpdater::selectReferencePatches(const State& state) {
  const int psize = cfg_.visual.patch_size;
  const int area = psize * psize;

  for (const auto& vp : submap_.points) {
    if (!vp || vp->obs.size() < 2) continue;

    // ---- 法向量精化（收敛后跳过，省计算）----
    if (!vp->normal_converged) {
      if (const VoxelPlane* pl = map_->queryPlane(vp->pos)) {
        if (pl->is_plane) {
          // 用平面协方差做 3σ 卡方检验 —— 几何模块算出的 plane_var
          // 被视觉模块直接复用，这是两个模块深度耦合的体现。
          Eigen::Matrix<double, 1, 6> J_nq;
          J_nq.block<1, 3>(0, 0) = (vp->pos - pl->center).transpose();
          J_nq.block<1, 3>(0, 3) = -pl->normal.transpose();
          const double sigma = (J_nq * pl->plane_var * J_nq.transpose())(0, 0) +
                               pl->normal.transpose() * vp->covariance * pl->normal;
          const double dis = std::fabs(pl->distanceTo(vp->pos));

          if (dis < 3.0 * std::sqrt(std::max(sigma, 1e-9))) {
            // 法向量有 ± 两个方向，用上一次的结果消歧
            V3D n = pl->normal;
            if (vp->normal_initialized && vp->prev_normal.dot(n) < 0) n = -n;
            const double update = (n - vp->prev_normal).norm();
            vp->normal = n;
            vp->prev_normal = n;
            vp->normal_initialized = true;
            if (update < 1e-4 && vp->obs.size() > 10) vp->normal_converged = true;
          }
        }
      }
    }

    // ---- 参考 patch 打分 ----
    float best_score = -1e9f;
    VisualPatchPtr best;

    for (const auto& cand : vp->obs) {
      if (cand->patch.size() != static_cast<size_t>(area)) continue;

      // 指标 1：观测角度
      V3D pf = (cand->rot_cw * vp->pos + cand->pos_cw).normalized();
      double cos_angle = 0.0;
      if (vp->normal_initialized) {
        const V3D n_cam = cand->rot_cw * vp->normal;
        cos_angle = std::fabs(pf.dot(n_cam));
      }

      // 指标 2：与其他观测的平均 NCC
      double ncc_sum = 0.0;
      int cnt = 0;
      double mean_a = 0;
      for (float v : cand->patch) mean_a += v;
      mean_a /= area;

      for (const auto& other : vp->obs) {
        if (other == cand || other->patch.size() != static_cast<size_t>(area)) continue;
        double mean_b = 0;
        for (float v : other->patch) mean_b += v;
        mean_b /= area;

        double num = 0, da = 0, db = 0;
        for (int k = 0; k < area; ++k) {
          const double a = cand->patch[k] - mean_a;
          const double b = other->patch[k] - mean_b;
          num += a * b; da += a * a; db += b * b;
        }
        if (da > 1e-6 && db > 1e-6) { ncc_sum += std::fabs(num / std::sqrt(da * db)); ++cnt; }
      }
      const double ncc = (cnt > 0) ? ncc_sum / cnt : 0.0;
      const float score = static_cast<float>(ncc + cos_angle);

      if (score > best_score) { best_score = score; best = cand; }
    }

    if (best) {
      best->score = best_score;
      vp->ref_patch = best;
      vp->has_ref_patch = true;
    }
  }
}

}  // namespace lvi
