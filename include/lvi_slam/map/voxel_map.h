// lvi_slam/map/voxel_map.h
// 统一体素地图：几何（平面）与视觉（图像块）共用同一套哈希索引。
//
// 这是整个系统耦合最深的地方，也是相对 LVI-SAM 那种"两个子系统互相提供初值"
// 架构的核心优势：给定一个视觉点，可以 O(1) 查到它所在体素的平面法向量和
// 法向量的不确定度，从而
//   1. 用精确的平面单应 H = R - t*n^T/d 做 patch 仿射变换（不用 fronto-parallel 假设）
//   2. 用平面协方差做法向量更新的卡方检验
#pragma once

#include "lvi_slam/common/types.h"
#include "lvi_slam/common/config.h"
#include <unordered_map>
#include <memory>

namespace lvi {

// ---------------------------------------------------------------------------
// 体素哈希键
// ---------------------------------------------------------------------------
struct VoxelKey {
  int64_t x = 0, y = 0, z = 0;
  bool operator==(const VoxelKey& o) const {
    return x == o.x && y == o.y && z == o.z;
  }
  static VoxelKey fromPoint(const V3D& p, double size) {
    VoxelKey k;
    k.x = static_cast<int64_t>(std::floor(p.x() / size));
    k.y = static_cast<int64_t>(std::floor(p.y() / size));
    k.z = static_cast<int64_t>(std::floor(p.z() / size));
    return k;
  }
};

struct VoxelKeyHash {
  size_t operator()(const VoxelKey& k) const {
    // 大质数混合，减少碰撞
    return static_cast<size_t>(((k.z * 116101) + k.y) * 116101 + k.x);
  }
};

// ---------------------------------------------------------------------------
// 体素平面：不只是 (n, d)，还带参数本身的协方差
//
// plane_var 是 6x6：法向量 3 维 + 中心点 3 维。
// 它让"刚建立的、只有几个点的平面"自动获得低权重，
// 而不像 LOAM/LIO-SAM 那样假设平面是精确的。
// ---------------------------------------------------------------------------
struct VoxelPlane {
  V3D center = V3D::Zero();
  V3D normal = V3D::Zero();
  M3D point_cov = M3D::Zero();
  Eigen::Matrix<double, 6, 6> plane_var =
      Eigen::Matrix<double, 6, 6>::Zero();

  double d = 0.0;
  double radius = 0.0;
  double min_eigen = 1.0, mid_eigen = 1.0, max_eigen = 1.0;
  bool   is_plane = false;
  int    points_num = 0;

  double distanceTo(const V3D& p) const {
    return normal.dot(p) + d;
  }
};

// ---------------------------------------------------------------------------
// 视觉点：3D 位置来自 LiDAR，法向量来自体素平面
// ---------------------------------------------------------------------------
struct VisualPatch;   // 前向声明，定义在 vio/visual_types.h

struct VisualPoint {
  V3D pos    = V3D::Zero();
  V3D normal = V3D::Zero();
  V3D prev_normal = V3D::Zero();
  M3D covariance  = M3D::Zero();

  std::vector<std::shared_ptr<VisualPatch>> obs;   // 多帧观测
  std::shared_ptr<VisualPatch> ref_patch;          // 当前选定的参考 patch

  bool normal_initialized = false;
  bool normal_converged   = false;   // 收敛后跳过更新，省计算
  bool has_ref_patch      = false;
};
using VisualPointPtr = std::shared_ptr<VisualPoint>;

// ---------------------------------------------------------------------------
// 自适应八叉树体素
// ---------------------------------------------------------------------------
class VoxelOctoTree {
 public:
  VoxelOctoTree(const VoxelMapConfig& cfg, int layer, int max_layer);

  // 体素的几何范围由 VoxelMap 在插入时设定（哈希键决定中心）
  void setGeometry(const V3D& center, double quater_length) {
    center_ = center; quater_length_ = quater_length;
  }
  ~VoxelOctoTree();

  void insertPoint(const PointWithVar& pv);
  void initOctoTree();                       // 首次拟合平面 / 决定是否细分
  void updateOctoTree(const PointWithVar& pv);

  // 找到包含 p 的最深层叶子节点
  const VoxelOctoTree* findCorrespond(const V3D& p) const;

  const VoxelPlane* plane() const { return plane_.get(); }

  // 视觉点挂载在体素上，和几何共用同一个空间划分
  std::vector<VisualPointPtr> visual_points;

 private:
  void fitPlane();
  int  childIndex(const V3D& p) const;
  void ensureChild(int idx, double child_quater);       // PCA 拟合平面 + 计算 plane_var
  void split();          // 细分成 8 个子体素

  VoxelMapConfig cfg_;
  int layer_ = 0, max_layer_ = 3;
  V3D center_ = V3D::Zero();
  double quater_length_ = 0.0;

  std::vector<PointWithVar> temp_points_;
  std::unique_ptr<VoxelPlane> plane_;
  VoxelOctoTree* children_[8] = {nullptr};
  bool is_leaf_ = true;
  bool init_done_ = false;
};

// ---------------------------------------------------------------------------
// 地图管理器
// ---------------------------------------------------------------------------
class VoxelMap {
 public:
  explicit VoxelMap(const Config& cfg);
  ~VoxelMap();

  void build(const std::vector<PointWithVar>& points);
  void update(const std::vector<PointWithVar>& points);

  // 为一批点找到对应平面并构建点-面残差。内部 OpenMP 并行。
  void buildResidualList(const std::vector<PointWithVar>& points,
                         std::vector<PointToPlane>* out) const;

  // 局部滑窗：删除远离当前位置的体素，保证内存有界。
  // 这是纯里程计前端的必然选择——全局地图交给后端的关键帧管理。
  void slideMap(const V3D& current_pos);

  // 回环后重建局部地图：用后端修正过的关键帧位姿重新灌点。
  // 这是解决 "FAST-LIO2 外接位姿图后地图改不了" 那个问题的关键接口。
  void rebuildFromKeyframes(const std::vector<KeyframePtr>& kfs);

  // 视觉模块查询接口：给一个 3D 点，返回它所在体素的平面
  const VoxelPlane* queryPlane(const V3D& point_w) const;

  // 视觉点的增删查
  void addVisualPoint(const VisualPointPtr& vp);
  void retrieveVisualPoints(const V3D& center, double radius,
                            std::vector<VisualPointPtr>* out) const;

  size_t voxelCount() const { return voxels_.size(); }

  // 逐点噪声：沿光束方向是测距误差，垂直方向是角分辨率误差
  // 远处点横向不确定度被距离放大，这是 LiDAR 噪声的物理本质
  static void calcPointCov(const V3D& p, double depth_err, double beam_err_deg,
                           M3D* cov);

 private:
  void clearOutOfRange(const VoxelKey& center_key);
  VoxelOctoTree* fetchOrCreate(const VoxelKey& key);

  Config cfg_;
  std::unordered_map<VoxelKey, VoxelOctoTree*, VoxelKeyHash> voxels_;
  V3D last_slide_pos_ = V3D::Zero();
  bool slide_initialized_ = false;
};

}  // namespace lvi
