# 只含官方 livox_ros_driver2 的消息定义，不含驱动节点，不依赖 Livox-SDK2。
# 同一份源码同时支持 ROS1 (catkin) 与 ROS2 (ament)，按 ROS_VERSION 环境变量选择。
# 用于回放 Mid-360 等 livox_ros_driver2 录制的 bag；接真实雷达时换成官方完整驱动，
# lvi_slam 无需改动。
cmake_minimum_required(VERSION 3.5)
project(livox_ros_driver2)

if("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
  add_message_files(FILES CustomPoint.msg CustomMsg.msg)
  generate_messages(DEPENDENCIES std_msgs)
  catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
else()
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    msg/CustomPoint.msg
    msg/CustomMsg.msg
    DEPENDENCIES std_msgs)
  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()
