# 只含官方 livox_ros_driver 的消息定义，不含驱动节点，不依赖 Livox-SDK。
# 用于回放 bag（MARS-LVIG、R3LIVE 等 Avia 数据集）。
# 接真实雷达时，用官方完整驱动替换本包即可，lvi_slam 无需改动。
cmake_minimum_required(VERSION 3.0.2)
project(livox_ros_driver)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES CustomPoint.msg CustomMsg.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
