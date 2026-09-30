# 数据集接入指南

## 为什么点云剖面必须显式配置

各家 LiDAR 的 `PointCloud2` 字段布局和时间语义都不同，**且时间语义错了不会崩溃，只表现为「精度差一点」**——这是最难排查的一类 bug。所以本系统拒绝自动猜测，必须显式指定 `--profile`。

| 剖面 | 时间字段 | 类型 | 语义 | 代表数据集 |
|---|---|---|---|---|
| `livox` | `offset_time` | uint32 | 相对帧首，纳秒 | MARS-LVIG, R3LIVE, 自采 |
| `ouster` | `t` | uint32 | 相对帧首，纳秒 | NTU VIRAL, Newer College |
| `velodyne` | `time` | float32 | 相对帧首，**秒** | 通用 Velodyne |
| `hesai` | `timestamp` | float64 | **绝对 UNIX 时间** | Hilti'22, Hilti'23 |
| `kitti` | 无 | — | 按方位角反推 | KITTI |

解码器带自检：单帧时间跨度不在 `[1ms, 500ms]` 区间会立刻警告。看到这个警告先查 profile，别查算法。

另一个隐蔽的坑：**用消息自带的 `header.stamp`，不是 bag 的接收时间戳**。两者可能差几十毫秒（驱动到 bag 的传输延迟），用错会让所有传感器的相对时序整体偏移。

---

## MARS-LVIG（推荐主力）

港大 MaRS Lab 的无人机数据集，21 个序列，覆盖航模机场、海岛、乡镇、山谷，飞行高度 80–130m，速度 3–12 m/s。传感器是 **Livox Avia + 内置 BMI088 IMU（200Hz）+ 相机 + GNSS**，LiDAR 10Hz。

**为什么它对本项目最重要**：唯一的 Livox + 下视视角公开数据集，`CustomMsg` 格式和 Mid-360 完全一致，改个话题名就能跑。

```bash
pip install rosbags
rosbags-convert --src HKisland01.bag --dst HKisland01_ros2

lvi_offline --config config/mars_lvig.yaml \
  --data HKisland01_ros2 --profile livox \
  --imu-topic /livox/imu --lidar-topic /livox/lidar \
  --image-topic /left_camera/image --out mars_hk01.tum
```

**必须改的参数**：无人机 100m 高空下视，用室内参数会因为体素太密而爆内存。

```yaml
voxel:
  max_voxel_size: 2.0      # 室内 0.5 → 高空 2.0
  half_map_size: 150
lidar:
  blind_max: 200.0         # 室内 70 → 高空 200
  filter_size: 0.5
```

---

## NTU VIRAL

南洋理工校园无人机数据集。**Ouster OS1-16（10Hz）+ 内置 IMU（100Hz）+ 两个同步针孔相机（10Hz）**，评测用左目。

价值在于退化场景：`sbs` 序列只能从远处物体获得噪声视觉特征；`nya` 序列有半透明表面（难倒激光）和复杂飞行动力学 + 低光照（难倒视觉）。拿来专门测混合视觉前端的切换逻辑非常合适。

```bash
lvi_offline --config config/ntu_viral.yaml --data eee_01_ros2 \
  --profile ouster --imu-topic /imu/imu \
  --lidar-topic /os1_cloud_node1/points --image-topic /left/image_raw
```

---

## Hilti SLAM Challenge（'22 / '23）

手持/机器人采集，建筑工地、办公室、实验室、停车场。**Hesai PandarXT-32 + Alphasense 五目 + Bosch BMI085 IMU（400Hz）**。8 个序列，难度来自恶劣光照或剧烈运动。

**注意两点**：

1. 时间字段是**绝对 UNIX 时间**，必须用 `--profile hesai`。用 velodyne 剖面会得到完全错误的去畸变
2. **真值不开源**，只能把轨迹上传官网评测。所以不适合日常迭代，适合最后对标国际水平

---

## KITTI（不推荐用于 LVIO）

只有 Velodyne + oxts，而 oxts 每帧一个文件、约 10Hz —— **紧耦合需要 100Hz 以上，KITTI 的 IMU 频率远远不够**。点云也没有逐点时间，只能按方位角反推，本身就有损。

加载器支持它只是为了冒烟测试（验证代码能跑通），不要用它的数字下结论。

---

## 接新数据集的五步

1. `ros2 bag info <bag>` 看话题名和消息类型
2. 若是 `PointCloud2`，看字段名判断用哪个 profile；字段名都对不上就在 `cloud_decoder.cpp` 里加一个新 profile（**不用改任何其他代码**）
3. 跑一遍，**先看时间跨度自检有没有报警**
4. 看 offline_runner 输出的实际频率统计，确认 IMU/点云比例 ≥ 5
5. 最后才看轨迹

前三步跳过任何一步，后面都是在浪费时间。

---

## 批量评测

单序列的数字没有意义 —— 参数调好可能只是过拟合了那一个序列。

```bash
./scripts/eval_batch.sh config/mars_lvig.yaml \
    /data/MARS-LVIG /data/MARS-LVIG/gt livox
```

脚本会把当次配置文件一起存进结果目录。**没有这个，三周后你看到一个好结果却复现不出来。**
