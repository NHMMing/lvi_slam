# Livox 点云消息。字段、类型、顺序必须与官方 livox_ros_driver2 一致（ROS1 MD5 / ROS2 类型哈希校验）
std_msgs/Header header    # ROS 标准消息头
uint64 timebase           # 首点时间，纳秒
uint32 point_num          # 点数
uint8  lidar_id           # 雷达编号
uint8[3]  rsvd            # 保留
CustomPoint[] points      # 点
