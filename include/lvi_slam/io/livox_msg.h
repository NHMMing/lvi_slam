// lvi_slam/io/livox_msg.h
// Livox CustomMsg 在不同 ROS 版本/驱动下的头文件和命名空间都不一样，
// 在这里统一成 lvi::LivoxCustomMsg，其余代码不用关心。
//
//   ROS1 + livox_ros_driver   : <livox_ros_driver/CustomMsg.h>        （Avia/Horizon，多数公开数据集）
//   ROS1 + livox_ros_driver2  : <livox_ros_driver2/CustomMsg.h>       （Mid-360）
//   ROS2 + livox_ros_driver2  : <livox_ros_driver2/msg/custom_msg.hpp>
//
// 三者的字段（x/y/z/reflectivity/tag/line/offset_time）完全一致。
// 宏由 CMake 定义：LVI_ROS1 / LVI_ROS2，ROS1 下可选 LVI_LIVOX_DRIVER2。
#pragma once

#if defined(LVI_ROS1)
  #if defined(LVI_LIVOX_DRIVER2)
    #include <livox_ros_driver2/CustomMsg.h>
    namespace lvi { using LivoxCustomMsg = livox_ros_driver2::CustomMsg; }
  #else
    #include <livox_ros_driver/CustomMsg.h>
    namespace lvi { using LivoxCustomMsg = livox_ros_driver::CustomMsg; }
  #endif
#elif defined(LVI_ROS2)
  #include <livox_ros_driver2/msg/custom_msg.hpp>
  namespace lvi { using LivoxCustomMsg = livox_ros_driver2::msg::CustomMsg; }
#else
  #error "livox_msg.h 需要定义 LVI_ROS1 或 LVI_ROS2"
#endif
