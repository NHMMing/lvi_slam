// tools/offline_runner.cpp
// 离线评测运行器。
//
// 这个工具的优先级高于任何算法新特性 —— 没有量化评估，
// 你不知道每次改动是变好还是变坏，所有优化都成了盲猜。
//
// 三条设计原则：
//   1. 严格按时间戳顺序 feed，复现在线时的消息到达序。
//      按传感器分别遍历会让 DataSynchronizer 走不同的路径，
//      离线跑得好好的、上车就出问题，且无法复现。
//   2. 默认全速跑（不 sleep），但支持 --realtime 模拟真实节奏，
//      用来暴露「算力不够导致丢帧」这类只在实时下出现的问题。
//   3. 分段计时。总耗时没有信息量，要知道时间花在哪。
#include "lvi_slam/system.h"
#include "lvi_slam/io/data_source.h"
#include "lvi_git_rev.h"

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>

namespace {

struct Args {
  std::string config;
  std::string data;
  std::string out;               // 只要 odometry 轨迹时用（兼容旧用法）
  std::string out_dir;           // 完整评估输出目录
  std::string profile = "livox";
  bool   realtime = false;
  double speed = 1.0;
  double start_offset = 0.0;    // 跳过前 N 秒（很多序列开头在标定/静置）
  double duration = -1.0;
  lvi::TopicMap topics;
};

void printUsage() {
  std::cout << R"(
用法: lvi_offline --config <yaml> --data <bag或目录> [选项]

必需:
  --config <path>       算法配置文件
  --data <path>         ROS2 bag（.db3/.mcap/目录）或 KITTI 风格文件夹

可选:
  --out-dir <dir>       评估输出目录（推荐），包含:
                          odometry.tum  前端逐帧在线位姿（含回环跳变）
                          slam.tum      逐帧位姿，按后端最终优化结果重新表达
                          keyframes.tum 后端关键帧最终位姿
                          frames.csv    逐帧滤波器诊断（耗时/残差/退化/协方差/bias/td）
                          loops.csv     回环候选及验证结果
                          resources.csv 内存与 CPU 采样
                          run_meta.json 版本、参数、统计；config_used.yaml 配置副本
  --out <path>          只输出 odometry 轨迹到指定文件（未给 --out-dir 时默认 trajectory.tum）
  --profile <name>      点云剖面: livox|ouster|velodyne|hesai|kitti（默认 livox）
  --imu-topic <name>    默认 /livox/imu
  --lidar-topic <name>  默认 /livox/lidar
  --image-topic <name>  默认 /left_camera/image，留空则不读图像
  --gps-topic <name>    默认空
  --realtime            按真实时间回放（暴露实时性问题），默认全速
  --speed <x>           配合 --realtime 的倍速，默认 1.0
  --start <sec>         跳过开头 N 秒
  --duration <sec>      只跑 N 秒

常用数据集示例:
  # MARS-LVIG (Livox Avia；ROS1 构建直接传 .bag，ROS2 构建需先 rosbags-convert)
  lvi_offline --config config/livox_mid360.yaml --data HKisland01_ros2 \
      --profile livox --imu-topic /livox/imu --lidar-topic /livox/lidar \
      --image-topic /left_camera/image

  # NTU VIRAL (Ouster OS1-16)
  lvi_offline --config config/ntu_viral.yaml --data eee_01_ros2 \
      --profile ouster --imu-topic /imu/imu --lidar-topic /os1_cloud_node1/points \
      --image-topic /left/image_raw

  # Hilti'22 (Hesai XT-32，注意时间字段是绝对时间)
  lvi_offline --config config/hilti.yaml --data exp01_ros2 \
      --profile hesai --imu-topic /alphasense/imu --lidar-topic /hesai/pandar

评估:
  lvi_eval run --gt groundtruth.tum --run <out-dir>
)" << std::endl;
}

bool parseArgs(int argc, char** argv, Args* a) {
  a->topics.imu = "/livox/imu";
  a->topics.lidar = "/livox/lidar";
  a->topics.image = "/left_camera/image";

  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto val = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };

    if      (k == "--config")      a->config = val();
    else if (k == "--data")        a->data = val();
    else if (k == "--out")         a->out = val();
    else if (k == "--out-dir")     a->out_dir = val();
    else if (k == "--profile")     a->profile = val();
    else if (k == "--imu-topic")   a->topics.imu = val();
    else if (k == "--lidar-topic") a->topics.lidar = val();
    else if (k == "--image-topic") a->topics.image = val();
    else if (k == "--gps-topic")   a->topics.gps = val();
    else if (k == "--realtime")    a->realtime = true;
    else if (k == "--speed")       a->speed = std::stod(val());
    else if (k == "--start")       a->start_offset = std::stod(val());
    else if (k == "--duration")    a->duration = std::stod(val());
    else if (k == "-h" || k == "--help") { printUsage(); return false; }
    else { std::cerr << "未知参数: " << k << "\n"; return false; }
  }
  if (a->out.empty() && a->out_dir.empty()) a->out = "trajectory.tum";
  return !a->config.empty() && !a->data.empty();
}

// 全速回放时允许的最大点云积压帧数（同步器缓冲上限是 50，远低于它即可）
constexpr size_t kMaxCloudBacklog = 5;

// 分段计时。总耗时没有信息量，要知道时间花在哪才能优化。
struct Profiler {
  struct Stat { double total = 0; double max = 0; size_t count = 0; };
  std::map<std::string, Stat> stats;

  void add(const std::string& name, double ms) {
    auto& s = stats[name];
    s.total += ms;
    s.max = std::max(s.max, ms);
    ++s.count;
  }

  void report() const {
    std::cout << "\n===== 耗时统计 =====\n"
              << std::left << std::setw(16) << "阶段"
              << std::right << std::setw(10) << "均值(ms)"
              << std::setw(10) << "最大(ms)"
              << std::setw(10) << "次数" << "\n";
    for (const auto& [name, s] : stats) {
      if (s.count == 0) continue;
      std::cout << std::left << std::setw(16) << name
                << std::right << std::fixed << std::setprecision(2)
                << std::setw(10) << s.total / s.count
                << std::setw(10) << s.max
                << std::setw(10) << s.count << "\n";
    }
  }
};

// 进程资源采样（Linux /proc）。内存曲线用来验证「局部地图有界」——
// 只看峰值看不出泄漏，要看随时间是否持续增长。
class ResourceSampler {
 public:
  struct Sample { double wall, data_t, rss_mb, cpu_pct; };
  ~ResourceSampler() { stop(); }

  void start(const std::atomic<double>* data_t) {
    data_t_ = data_t;
    running_ = true;
    th_ = std::thread([this] { loop(); });
  }
  void stop() {
    running_ = false;
    if (th_.joinable()) th_.join();
  }
  const std::vector<Sample>& samples() const { return samples_; }
  double peakRssMb() const {
    double m = 0;
    for (const auto& s : samples_) m = std::max(m, s.rss_mb);
    return m;
  }

 private:
  static double rssMb() {
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line)) {
      if (line.rfind("VmRSS:", 0) == 0) return std::stod(line.substr(6)) / 1024.0;
    }
    return 0;
  }
  static double cpuSec() {   // utime + stime，所有线程合计
    std::ifstream f("/proc/self/stat");
    std::string content((std::istreambuf_iterator<char>(f)), {});
    std::istringstream is(content.substr(content.rfind(')') + 2));
    std::string tok;
    double ut = 0, st = 0;
    for (int field = 3; is >> tok; ++field) {
      if (field == 14) ut = std::stod(tok);
      if (field == 15) { st = std::stod(tok); break; }
    }
    return (ut + st) / static_cast<double>(sysconf(_SC_CLK_TCK));
  }
  void loop() {
    const auto t0 = std::chrono::steady_clock::now();
    double last_wall = 0, last_cpu = cpuSec();
    while (running_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      const double wall = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0).count();
      const double cpu = cpuSec();
      samples_.push_back({wall, data_t_->load(), rssMb(),
                          100.0 * (cpu - last_cpu) / std::max(1e-6, wall - last_wall)});
      last_wall = wall;
      last_cpu = cpu;
    }
  }

  const std::atomic<double>* data_t_ = nullptr;
  std::atomic<bool> running_{false};
  std::thread th_;
  std::vector<Sample> samples_;
};

std::string jsonEscape(const std::string& in) {
  std::string out;
  for (char c : in) {
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out;
}

std::string cpuModel() {
  std::ifstream f("/proc/cpuinfo");
  std::string line;
  while (std::getline(f, line)) {
    // x86: "model name"；ARM（Orin）没有 model name，退而取 "Hardware"/"CPU part"
    if (line.rfind("model name", 0) == 0 || line.rfind("Hardware", 0) == 0) {
      return line.substr(line.find(':') + 2);
    }
  }
  return "unknown";
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parseArgs(argc, argv, &args)) { printUsage(); return 1; }

  lvi::Config cfg;
  if (!cfg.loadFromFile(args.config)) return 1;

  lvi::CloudProfile profile;
  if (!lvi::CloudProfile::fromName(args.profile, &profile)) {
    std::cerr << "未知的点云剖面: " << args.profile << "\n";
    return 1;
  }

  auto source = lvi::DataSource::create(args.data, cfg, args.topics, profile);
  if (!source) return 1;

  if (!args.out_dir.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(args.out_dir, ec);
    if (ec) { std::cerr << "无法创建输出目录 " << args.out_dir << ": " << ec.message() << "\n"; return 1; }
  }

  lvi::System system(cfg);
  if (!args.out_dir.empty()) system.enableDiagnostics(true);
  system.start();

  std::atomic<double> progress_t{0.0};   // 当前喂到的数据时间（相对首条消息）
  ResourceSampler sampler;
  sampler.start(&progress_t);

  Profiler prof;
  // --start / --duration 以第一条消息的 header.stamp 为零点，而不是 bag 的录制时间。
  // 很多设备（如 Livox Avia 未接 PTP/GPS 时）header.stamp 是开机时钟（几百秒），
  // 而 bag 时间是 Unix 时间，两者直接比较会把所有消息都判成「早于起点」。
  // source 的 start/end 只用来拿总时长（同一时钟域内的差值，与时钟基准无关）。
  const double total_span = source->endTime() - source->startTime();
  double t_begin = 0, t_end = 0;
  bool time_anchored = false;
  double last_msg_time = 0;

  size_t n_imu = 0, n_cloud = 0, n_img = 0, n_gps = 0, n_skipped = 0;
  auto wall_start = std::chrono::steady_clock::now();
  double first_msg_time = -1;

  lvi::SourceMessage msg;
  while (source->next(&msg)) {
    if (!time_anchored) {
      t_begin = msg.t + args.start_offset;
      t_end = (args.duration > 0) ? t_begin + args.duration
                                  : std::numeric_limits<double>::infinity();
      time_anchored = true;
    }
    if (msg.t < t_begin) { ++n_skipped; continue; }
    if (msg.t > t_end) break;
    last_msg_time = msg.t;
    if (first_msg_time < 0) first_msg_time = msg.t;
    progress_t = msg.t - first_msg_time;

    // 实时模式：按数据时间戳节奏 sleep。
    // 全速模式下系统永远不会「跟不上」，很多实时性 bug 会被掩盖。
    if (args.realtime) {
      const double data_elapsed = (msg.t - first_msg_time) / args.speed;
      const double wall_elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - wall_start).count();
      if (data_elapsed > wall_elapsed) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(data_elapsed - wall_elapsed));
      }
    }

    // 全速模式的背压：前端积压过多就等它消化，而不是让同步器缓冲溢出丢帧。
    // 只在喂点云前等 —— 期间 IMU 照常喂，保证积压的帧都有 IMU 覆盖，不会死锁。
    // 阈值至少为 2：同步一帧需要 IMU 越过帧尾，也就是下一帧已经开始喂了。
    if (!args.realtime && msg.type == lvi::MsgType::kCloud) {
      while (system.pendingClouds() >= kMaxCloudBacklog) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }

    const auto tic = std::chrono::steady_clock::now();
    switch (msg.type) {
      case lvi::MsgType::kImu:
        system.feedImu(msg.imu); ++n_imu; break;
      case lvi::MsgType::kCloud:
        system.feedCloud(msg.cloud, msg.t); ++n_cloud; break;
      case lvi::MsgType::kImage:
        system.feedImage(msg.image, msg.t); ++n_img; break;
      case lvi::MsgType::kGps:
        system.feedGps(msg.t, msg.gps_pos, msg.gps_cov); ++n_gps; break;
      default: break;
    }
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - tic).count();

    if (msg.type == lvi::MsgType::kCloud) prof.add("feed_cloud", ms);
    else if (msg.type == lvi::MsgType::kImage) prof.add("feed_image", ms);

    // 进度条
    if ((n_cloud + n_img) % 50 == 0 && msg.type != lvi::MsgType::kImu) {
      const double span = (args.duration > 0) ? args.duration
                                               : total_span - args.start_offset;
      const double pct = 100.0 * (msg.t - t_begin) / std::max(1e-6, span);
      std::cout << "\r进度 " << std::fixed << std::setprecision(1) << pct
                << "%  点云 " << n_cloud << "  图像 " << n_img
                << "  IMU " << n_imu << std::flush;
    }
  }
  std::cout << "\n";

  // 全速模式下 feed 完不代表算完 —— 前端还在消费队列，等它排空
  // 最后一帧点云可能因为没有后续 IMU 永远凑不齐，所以不能等到积压为 0，
  // 而是等到「积压数不再变化且前端空闲」持续一段时间。
  std::cout << "等待前端处理完剩余队列...\n";
  {
    size_t last = system.pendingClouds();
    auto last_change = std::chrono::steady_clock::now();
    while (last > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      const size_t now = system.pendingClouds();
      if (now != last) { last = now; last_change = std::chrono::steady_clock::now(); continue; }
      if (std::chrono::steady_clock::now() - last_change > std::chrono::milliseconds(500)) break;
    }
    if (last > 0) std::cout << "  " << last << " 帧点云因数据末尾缺少 IMU 未能处理\n";
  }
  std::cout << "等待后端完成最终优化...\n";
  system.finish();
  sampler.stop();

  const double wall_total = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - wall_start).count();
  const double data_span = (first_msg_time < 0) ? 0.0 : last_msg_time - first_msg_time;

  std::cout << "\n===== 回放统计 =====\n"
            << "数据时长: " << std::fixed << std::setprecision(2) << data_span << " s\n"
            << "墙钟耗时: " << wall_total << " s\n"
            << "实时倍率: " << (wall_total > 0 ? data_span / wall_total : 0) << "x"
            << (data_span > wall_total ? "  (快于实时 ✓)" : "  (慢于实时 ✗)") << "\n"
            << "消息数: IMU=" << n_imu << " 点云=" << n_cloud
            << " 图像=" << n_img << " GPS=" << n_gps
            << " 跳过=" << n_skipped << "\n";
  const auto drops = system.dropStats();
  if (drops.imu + drops.cloud + drops.image > 0) {
    std::cerr << "[警告] 缓冲溢出丢弃: IMU=" << drops.imu << " 点云=" << drops.cloud
              << " 图像=" << drops.image << "（前端跟不上数据速率）\n";
  }
  std::cout << "峰值内存: " << std::setprecision(1) << sampler.peakRssMb() << " MB\n"
            << std::setprecision(2);

  // 数据完整性自检。这几个比率不对，说明话题名配错或者 bag 本身有问题。
  if (n_cloud > 0) {
    const double imu_rate = n_imu / std::max(1e-6, data_span);
    const double lidar_rate = n_cloud / std::max(1e-6, data_span);
    std::cout << "实际频率: IMU=" << imu_rate << "Hz  点云=" << lidar_rate << "Hz\n";
    if (imu_rate < 50) {
      std::cerr << "[警告] IMU 频率低于 50Hz，紧耦合会不稳。检查 --imu-topic\n";
    }
    if (n_imu / std::max<size_t>(n_cloud, 1) < 5) {
      std::cerr << "[警告] IMU/点云 比例过低，去畸变和传播精度会受影响\n";
    }
  } else {
    std::cerr << "[错误] 一帧点云都没读到。检查 --lidar-topic 和 --profile\n";
    return 1;
  }

  prof.report();

  if (!args.out.empty()) system.saveTrajectoryTUM(args.out);
  if (args.out_dir.empty()) {
    std::cout << "\n轨迹已保存: " << args.out << "\n";
    return 0;
  }

  namespace fs = std::filesystem;
  const fs::path dir(args.out_dir);
  system.saveTrajectoryTUM((dir / "odometry.tum").string());
  system.saveSlamTrajectoryTUM((dir / "slam.tum").string());
  system.saveKeyframesTUM((dir / "keyframes.tum").string());
  system.saveFrameDiagCSV((dir / "frames.csv").string());
  system.saveLoopEventsCSV((dir / "loops.csv").string());
  {
    std::ofstream f(dir / "resources.csv");
    f << "wall_s,data_s,rss_mb,cpu_pct\n";
    for (const auto& r : sampler.samples()) {
      f << r.wall << "," << r.data_t << "," << r.rss_mb << "," << r.cpu_pct << "\n";
    }
  }
  std::error_code ec;
  fs::copy_file(args.config, dir / "config_used.yaml",
                fs::copy_options::overwrite_existing, ec);

  {
    char host[256] = {0};
    gethostname(host, sizeof(host) - 1);
    const std::time_t now = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%S", std::localtime(&now));
    std::string argv_str;
    for (int i = 0; i < argc; ++i) argv_str += (i ? " " : "") + std::string(argv[i]);

    std::ofstream f(dir / "run_meta.json");
    f << std::setprecision(6) << "{\n"
      << "  \"git_rev\": \"" << LVI_GIT_REV << "\",\n"
      << "  \"build_type\": \"" << LVI_BUILD_TYPE << "\",\n"
      << "  \"date\": \"" << stamp << "\",\n"
      << "  \"host\": \"" << jsonEscape(host) << "\",\n"
      << "  \"cpu\": \"" << jsonEscape(cpuModel()) << "\",\n"
      << "  \"nproc\": " << std::thread::hardware_concurrency() << ",\n"
      << "  \"command\": \"" << jsonEscape(argv_str) << "\",\n"
      << "  \"config\": \"" << jsonEscape(args.config) << "\",\n"
      << "  \"data\": \"" << jsonEscape(args.data) << "\",\n"
      << "  \"profile\": \"" << args.profile << "\",\n"
      << "  \"realtime\": " << (args.realtime ? "true" : "false") << ",\n"
      << "  \"speed\": " << args.speed << ",\n"
      << "  \"data_start\": " << std::fixed << std::setprecision(9) << first_msg_time << ",\n"
      << std::defaultfloat << std::setprecision(6)
      << "  \"data_span_s\": " << data_span << ",\n"
      << "  \"wall_s\": " << wall_total << ",\n"
      << "  \"realtime_factor\": " << (wall_total > 0 ? data_span / wall_total : 0) << ",\n"
      << "  \"peak_rss_mb\": " << sampler.peakRssMb() << ",\n"
      << "  \"messages\": {\"imu\": " << n_imu << ", \"cloud\": " << n_cloud
      << ", \"image\": " << n_img << ", \"gps\": " << n_gps << "},\n"
      << "  \"dropped\": {\"imu\": " << drops.imu << ", \"cloud\": " << drops.cloud
      << ", \"image\": " << drops.image << "}\n"
      << "}\n";
  }

  std::cout << "\n评估数据已保存到 " << args.out_dir << "\n"
            << "评估命令:\n"
            << "  lvi_eval run --gt <groundtruth> --run " << args.out_dir << "\n";
  return 0;
}
