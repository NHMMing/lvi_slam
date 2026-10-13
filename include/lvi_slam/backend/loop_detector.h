// lvi_slam/backend/loop_detector.h
// 回环检测：Scan Context（激光）+ 视觉词袋 双模态验证。
//
// 为什么不用 LIO-SAM 的半径搜索：
//   它基于当前位姿在历史关键帧位置中做 KD-Tree 半径搜索，
//   依赖里程计精度——漂移超过搜索半径就检测不到。
//   这是个死循环：越需要回环的时候（漂移大）越检测不到。
//   Scan Context 是场景描述子，不依赖位置先验。
//
// 为什么要双模态：
//   误回环对因子图是灾难性的，一条错误的边能把整条轨迹拧坏。
//   激光和视觉的失效场景不重叠（激光怕结构重复，视觉怕光照变化），
//   两者都认为是回环才接受，能大幅压低误检率。
#pragma once

#include "lvi_slam/common/types.h"
#include "lvi_slam/common/config.h"
#include <opencv2/features2d.hpp>

namespace lvi {

struct LoopConstraint {
  int    id_from = -1;      // 当前帧
  int    id_to   = -1;      // 历史帧
  M3D    rot_delta = M3D::Identity();
  V3D    pos_delta = V3D::Zero();
  double noise = 1.0;       // 用 ICP fitness 自适应，匹配越好噪声越小
  bool   verified_lidar  = false;
  bool   verified_visual = false;

  // 诊断：本次检索走到了哪一步（通过 SC 阈值之后才有意义）
  bool   sc_hit = false;
  double sc_dist = -1.0;
  double icp_fitness = -1.0;
};

class LoopDetector {
 public:
  explicit LoopDetector(const Config& cfg);

  // 把关键帧加入检索库（只有 is_loop_candidate 的才进）
  void addKeyframe(const KeyframePtr& kf);

  // 检测回环。返回 true 表示找到并通过了几何验证。
  bool detect(const KeyframePtr& current,
              const std::vector<KeyframePtr>& all_kfs,
              LoopConstraint* out);

 private:
  // ---- Scan Context ----
  // 把点云编码成 ring x sector 的二维矩阵，每个 bin 存最大高度。
  // 旋转不变性通过 ring key（对 sector 维度求均值）实现。
  Eigen::MatrixXf makeScanContext(const CloudPtr& cloud) const;
  Eigen::VectorXf makeRingKey(const Eigen::MatrixXf& sc) const;
  double scDistance(const Eigen::MatrixXf& a, const Eigen::MatrixXf& b,
                    int* best_shift) const;

  // ---- 几何验证：ICP ----
  // 当前帧 vs 历史帧前后 N 帧拼成的子图（单帧点太稀，配不准）
  bool verifyByIcp(const KeyframePtr& cur, const KeyframePtr& prev,
                   const std::vector<KeyframePtr>& all_kfs,
                   LoopConstraint* out);

  // ---- 视觉验证 ----
  bool verifyByVisual(const KeyframePtr& cur, const KeyframePtr& prev);

  Config cfg_;

  int num_ring_   = 20;
  int num_sector_ = 60;
  double max_radius_ = 80.0;

  std::vector<Eigen::MatrixXf> sc_database_;
  std::vector<Eigen::VectorXf> ringkey_database_;
  std::vector<int>             kf_id_database_;

  cv::Ptr<cv::DescriptorMatcher> matcher_;
  int    last_query_id_ = -1;     // 防止同一帧重复检索
  double sc_yaw_init_   = 0.0;    // SC 移位量转成的偏航角，作为 ICP 初值
};

}  // namespace lvi
