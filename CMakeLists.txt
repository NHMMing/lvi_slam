cmake_minimum_required(VERSION 3.16)
project(lvi_slam)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wno-unused-parameter)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# -march=native 能带来可观的 Eigen 加速，但会让二进制不可跨机分发。
# 默认开启，交叉编译或做 Docker 镜像时用 -DNATIVE_ARCH=OFF 关掉。
option(NATIVE_ARCH "Enable -march=native" ON)
if(NATIVE_ARCH AND CMAKE_BUILD_TYPE STREQUAL "Release")
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-march=native" HAS_NATIVE)
  if(HAS_NATIVE)
    add_compile_options(-march=native)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP)

# ---------------------------------------------------------------------------
# 核心库：不依赖 ROS。
# 这样做的好处是核心算法可以脱离 ROS 单测、可以跑离线数据集、
# 也方便以后迁到其他中间件。ROS 只是一层薄薄的消息转换。
# ---------------------------------------------------------------------------
add_library(lvi_core SHARED
  src/common/config.cpp
  src/io/cloud_decoder.cpp
  src/io/data_source.cpp
  src/io/ros2bag_source.cpp
  src/preprocess/livox_preprocess.cpp
  src/estimator/iekf.cpp
  src/imu/imu_processor.cpp
  src/map/voxel_map.cpp
  src/lio/lio_updater.cpp
  src/vio/camera_model.cpp
  src/vio/direct_visual_updater.cpp
  src/vio/feature_visual_updater.cpp
  src/vio/hybrid_visual_updater.cpp
  src/sync/data_synchronizer.cpp
  src/frontend/frontend.cpp
  src/backend/loop_detector.cpp
  src/backend/backend.cpp
  src/system.cpp
)
target_include_directories(lvi_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(lvi_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  gtsam
  yaml-cpp
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(lvi_core OpenMP::OpenMP_CXX)
endif()

# ---------------------------------------------------------------------------
# ROS2 节点
# ---------------------------------------------------------------------------
add_executable(lvi_slam_node ros2/lvi_slam_node.cpp)
target_link_libraries(lvi_slam_node lvi_core)
ament_target_dependencies(lvi_slam_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros cv_bridge
  pcl_conversions livox_ros_driver2
)

# ---------------------------------------------------------------------------
# 离线运行器：无 ROS，直接跑数据集。评估基础设施优先级高于任何新特性。
# ---------------------------------------------------------------------------
add_executable(lvi_offline tools/offline_runner.cpp)
target_link_libraries(lvi_offline lvi_core)

# 离线工具默认带 rosbag2 支持。无 ROS 环境编译时关掉，
# 此时只能读 KITTI 风格文件夹。
option(LVI_WITH_ROSBAG2 "Enable rosbag2 reading in offline tool" ON)
if(LVI_WITH_ROSBAG2)
  find_package(rosbag2_cpp REQUIRED)
  find_package(rosbag2_storage REQUIRED)
  target_compile_definitions(lvi_core PRIVATE LVI_WITH_ROSBAG2)
  ament_target_dependencies(lvi_core
    rosbag2_cpp rosbag2_storage rclcpp sensor_msgs cv_bridge livox_ros_driver2)
endif()

install(TARGETS lvi_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS lvi_slam_node lvi_offline DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core lvi_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(lvi_core)
ament_package()
