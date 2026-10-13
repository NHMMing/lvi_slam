# livox_ros_driver2（仅消息定义）

本目录的 `msg/CustomMsg.msg`、`msg/CustomPoint.msg` 取自 Livox 官方
[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)，
该项目以 MIT 许可证发布，版权归 Livox Technology 所有。

这里只保留消息定义，同时支持 ROS1（catkin）与 ROS2（ament），用于在没有 Livox-SDK2 的
环境下回放 Mid-360 等设备录制的 bag；不包含驱动节点和 SDK。接真实雷达时请使用官方完整驱动。
