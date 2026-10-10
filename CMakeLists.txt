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

# ---------------------------------------------------------------------------
# ROS 版本选择：同一份源码同时支持 ROS1 (catkin) 和 ROS2 (ament)。
#   LVI_ROS_VERSION = AUTO（默认，读环境变量 ROS_VERSION）| 1 | 2 | 0（纯 CMake，无 ROS）
#   LVI_LIVOX_DRIVER = livox_ros_driver | livox_ros_driver2
#       仅 ROS1 有意义。Avia/Horizon 及多数公开数据集用 livox_ros_driver，
#       Mid-360 用 livox_ros_driver2。ROS2 只有 livox_ros_driver2。
# ---------------------------------------------------------------------------
set(LVI_ROS_VERSION "AUTO" CACHE STRING "ROS 版本: AUTO | 1 | 2 | 0")
set_property(CACHE LVI_ROS_VERSION PROPERTY STRINGS AUTO 1 2 0)
set(LVI_LIVOX_DRIVER "livox_ros_driver" CACHE STRING "ROS1 下的 Livox 驱动包名")
set_property(CACHE LVI_LIVOX_DRIVER PROPERTY STRINGS livox_ros_driver livox_ros_driver2)

# 不从 PATH 推导搜索前缀：conda 激活后 ~/miniconda3/bin 在 PATH 里，
# CMake 会把 ~/miniconda3 当成安装前缀，优先于 /usr 找到 conda 编的
# yaml-cpp、GTest 等，链接时出 GLIBCXX 版本错误。
# ROS 走 CMAKE_PREFIX_PATH 环境变量，/usr/local 在系统前缀里，都不受影响。
set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH OFF)

# 依赖装在非系统目录（如 ~/library/gtsam/install）且不跑 ldconfig 时，
# 让 install 出去的二进制也带上依赖库的 RPATH，否则运行时找不到 .so
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

if(LVI_ROS_VERSION STREQUAL "AUTO")
  if("$ENV{ROS_VERSION}" STREQUAL "1")
    set(LVI_ROS 1)
  elseif("$ENV{ROS_VERSION}" STREQUAL "2")
    set(LVI_ROS 2)
  else()
    set(LVI_ROS 0)
  endif()
else()
  set(LVI_ROS ${LVI_ROS_VERSION})
endif()
message(STATUS "lvi_slam: ROS_VERSION = ${LVI_ROS}")

if(LVI_ROS EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp rosbag sensor_msgs nav_msgs geometry_msgs tf2_ros cv_bridge
    pcl_conversions ${LVI_LIVOX_DRIVER})
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES lvi_core
    CATKIN_DEPENDS roscpp rosbag sensor_msgs nav_msgs geometry_msgs tf2_ros
                   cv_bridge pcl_conversions ${LVI_LIVOX_DRIVER})
elseif(LVI_ROS EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(livox_ros_driver2 REQUIRED)
endif()

# ---------------------------------------------------------------------------
# 第三方库版本锁定
#
# 机器上常同时存在多份同名库（apt、/usr/local 源码安装、conda），
# find_package 默认「找到哪个用哪个」，结果可能是 CMake 报 Eigen 3.4.0、
# 实际编译却 include 了 PCL 带进来的 3.3.7。这里把版本显式钉死，
# 并在配置阶段校验所有依赖用的是同一份 Eigen。
#
# 默认值对应 ROS1 Noetic / Ubuntu 20.04 的 apt 版本 —— Noetic 的二进制包
# （cv_bridge、pcl_conversions）就是用这一套编的，换不了。
# ROS2 下默认不锁（各发行版不同），需要时用 -DLVI_xxx_VERSION=... 指定；
# 设为空字符串表示不锁定该库。
# ---------------------------------------------------------------------------
if(LVI_ROS EQUAL 2)
  set(_lvi_def_eigen "")
  set(_lvi_def_pcl "")
  set(_lvi_def_opencv "")
  set(_lvi_def_yaml "")
else()
  set(_lvi_def_eigen "3.3.7")
  set(_lvi_def_pcl "1.10")
  set(_lvi_def_opencv "4.2")
  set(_lvi_def_yaml "0.6.2")
endif()
set(LVI_EIGEN_VERSION    "${_lvi_def_eigen}"  CACHE STRING "Eigen 精确版本（空 = 不锁定）")
set(LVI_PCL_VERSION      "${_lvi_def_pcl}"    CACHE STRING "PCL major.minor（空 = 不锁定）")
set(LVI_OPENCV_VERSION   "${_lvi_def_opencv}" CACHE STRING "OpenCV major.minor（空 = 不锁定）")
set(LVI_YAML_CPP_VERSION "${_lvi_def_yaml}"   CACHE STRING "yaml-cpp 精确版本（空 = 不锁定）")
set(LVI_GTSAM_VERSION    "4.2"                CACHE STRING "GTSAM major.minor（空 = 不锁定）")

# 从 Eigen 头文件读真实版本号（不信任 CMake 包里报的版本）
function(lvi_eigen_version_of include_dir out_var)
  set(_h "${include_dir}/Eigen/src/Core/util/Macros.h")
  if(NOT EXISTS "${_h}")
    set(${out_var} "" PARENT_SCOPE)
    return()
  endif()
  file(STRINGS "${_h}" _l REGEX "#define EIGEN_(WORLD|MAJOR|MINOR)_VERSION[ \t]+[0-9]+")
  string(REGEX REPLACE ".*WORLD_VERSION[ \t]+([0-9]+).*" "\\1" _w "${_l}")
  string(REGEX REPLACE ".*MAJOR_VERSION[ \t]+([0-9]+).*" "\\1" _m "${_l}")
  string(REGEX REPLACE ".*MINOR_VERSION[ \t]+([0-9]+).*" "\\1" _n "${_l}")
  set(${out_var} "${_w}.${_m}.${_n}" PARENT_SCOPE)
endfunction()

# 校验 major.minor 一致（find_package 的版本参数只表示「至少」）
function(lvi_require_version name found want)
  if(want STREQUAL "")
    return()
  endif()
  string(REGEX MATCH "^[0-9]+\\.[0-9]+" _found_mm "${found}")
  if(NOT _found_mm VERSION_EQUAL want)
    message(FATAL_ERROR "lvi_slam: ${name} 期望 ${want}.x，实际找到 ${found}。"
      "用 -D${name}_DIR=... 指定正确的安装，或改 LVI_*_VERSION。")
  endif()
endfunction()

# Eigen：EXACT 让 find_package 跳过 /usr/local 下的 3.4.0，继续搜到 /usr 的 3.3.7
if(LVI_EIGEN_VERSION STREQUAL "")
  find_package(Eigen3 REQUIRED NO_MODULE)
else()
  find_package(Eigen3 ${LVI_EIGEN_VERSION} EXACT REQUIRED NO_MODULE)
endif()
get_target_property(LVI_EIGEN_INCLUDE_DIR Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
list(GET LVI_EIGEN_INCLUDE_DIR 0 LVI_EIGEN_INCLUDE_DIR)
lvi_eigen_version_of("${LVI_EIGEN_INCLUDE_DIR}" LVI_EIGEN_REAL_VERSION)

find_package(PCL ${LVI_PCL_VERSION} REQUIRED COMPONENTS common io filters kdtree registration)
lvi_require_version(PCL "${PCL_VERSION}" "${LVI_PCL_VERSION}")
find_package(OpenCV ${LVI_OPENCV_VERSION} REQUIRED)
lvi_require_version(OpenCV "${OpenCV_VERSION}" "${LVI_OPENCV_VERSION}")
# yaml-cpp：EXACT 让 find_package 跳过 conda 里的 0.8.0
if(LVI_YAML_CPP_VERSION STREQUAL "")
  find_package(yaml-cpp REQUIRED)
else()
  find_package(yaml-cpp ${LVI_YAML_CPP_VERSION} EXACT REQUIRED)
endif()
find_package(GTSAM ${LVI_GTSAM_VERSION} REQUIRED)
lvi_require_version(GTSAM "${GTSAM_VERSION}" "${LVI_GTSAM_VERSION}")
find_package(OpenMP)

# conda 的库是另一套工具链编的，混进来会出 libstdc++/ABI 问题
foreach(_pkg Eigen3 PCL OpenCV yaml-cpp GTSAM)
  if("${${_pkg}_DIR}" MATCHES "conda")
    message(FATAL_ERROR "lvi_slam: ${_pkg} 来自 conda（${${_pkg}_DIR}）。"
      "请先 conda deactivate 再重新配置（并清掉 build 目录）。")
  endif()
endforeach()

# 所有依赖暴露出来的 Eigen 头文件必须是同一个版本。
# 只看 include 路径里的 Eigen —— 头文件模板会在本工程里实例化，
# 版本不一致就是 ODR 违规，表现为难以复现的段错误。
get_target_property(_gtsam_incs gtsam INTERFACE_INCLUDE_DIRECTORIES)
if(NOT _gtsam_incs)
  set(_gtsam_incs "")
endif()
set(_eigen_sources "Eigen3::Eigen=${LVI_EIGEN_INCLUDE_DIR}")
foreach(_d ${PCL_INCLUDE_DIRS})
  list(APPEND _eigen_sources "PCL=${_d}")
endforeach()
foreach(_d ${_gtsam_incs})
  list(APPEND _eigen_sources "GTSAM=${_d}")
endforeach()
foreach(_d ${catkin_INCLUDE_DIRS})
  list(APPEND _eigen_sources "catkin=${_d}")
endforeach()
foreach(_s ${_eigen_sources})
  string(REGEX REPLACE "^([^=]+)=(.*)$" "\\1" _who "${_s}")
  string(REGEX REPLACE "^([^=]+)=(.*)$" "\\2" _d "${_s}")
  # 生成器表达式（如 $<BUILD_INTERFACE:...>）在配置期无法求值，跳过
  if(_d MATCHES "^\\$<")
    continue()
  endif()
  # 编译器默认搜索路径（如 /usr/local/include）CMake 不会生成 -I，
  # 优先级低于显式 -I 的 Eigen 目录，不会被选中，跳过
  list(FIND CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES "${_d}" _implicit)
  if(NOT _implicit EQUAL -1)
    continue()
  endif()
  foreach(_cand "${_d}" "${_d}/eigen3" "${_d}/gtsam/3rdparty/Eigen")
    lvi_eigen_version_of("${_cand}" _v)
    if(_v AND NOT _v STREQUAL LVI_EIGEN_REAL_VERSION)
      message(FATAL_ERROR "lvi_slam: Eigen 版本冲突 —— 本工程用 ${LVI_EIGEN_REAL_VERSION}"
        "（${LVI_EIGEN_INCLUDE_DIR}），但 ${_who} 带来了 ${_v}（${_cand}）。"
        "GTSAM 需用同一份 Eigen 重新编译：-DGTSAM_USE_SYSTEM_EIGEN=ON "
        "-DEIGEN3_INCLUDE_DIR=${LVI_EIGEN_INCLUDE_DIR}")
    endif()
  endforeach()
endforeach()

# -march=native 必须两边一致，否则 Eigen 固定大小类型的对齐要求不同，
# 跨库传递 Pose3/Matrix 时会崩在 aligned_allocator 或 ISAM2 内部。
get_target_property(_gtsam_opts gtsam INTERFACE_COMPILE_OPTIONS)
if(_gtsam_opts MATCHES "march=native")
  set(_gtsam_native ON)
else()
  set(_gtsam_native OFF)
endif()
if(NATIVE_ARCH AND CMAKE_BUILD_TYPE STREQUAL "Release")
  set(_lvi_native ON)
else()
  set(_lvi_native OFF)
endif()
if(NOT _gtsam_native STREQUAL _lvi_native)
  message(FATAL_ERROR "lvi_slam: -march=native 不一致（GTSAM=${_gtsam_native}，"
    "lvi_slam=${_lvi_native}）。重编 GTSAM 时设 -DGTSAM_BUILD_WITH_MARCH_NATIVE=${_lvi_native}，"
    "或本工程设 -DNATIVE_ARCH=${_gtsam_native}。")
endif()

message(STATUS "lvi_slam 依赖版本:")
message(STATUS "  Eigen    ${LVI_EIGEN_REAL_VERSION}  (${LVI_EIGEN_INCLUDE_DIR})")
message(STATUS "  PCL      ${PCL_VERSION}  (${PCL_DIR})")
message(STATUS "  OpenCV   ${OpenCV_VERSION}  (${OpenCV_DIR})")
message(STATUS "  yaml-cpp ${yaml-cpp_VERSION}  (${yaml-cpp_DIR})")
message(STATUS "  GTSAM    ${GTSAM_VERSION}  (${GTSAM_DIR})  march=native: ${_gtsam_native}")

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
  src/io/rosbag1_source.cpp
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
# 编译期再校验一次：types.h 里 static_assert 实际 include 到的 Eigen 版本。
# 配置期检查只能看 include 路径，这里兜住路径顺序导致的意外。
string(REPLACE "." ";" _ev "${LVI_EIGEN_REAL_VERSION}")
list(GET _ev 0 _ev0)
list(GET _ev 1 _ev1)
list(GET _ev 2 _ev2)
math(EXPR _ev_num "${_ev0} * 10000 + ${_ev1} * 100 + ${_ev2}")
target_compile_definitions(lvi_core PUBLIC LVI_EXPECTED_EIGEN_VERSION=${_ev_num})

# ---------------------------------------------------------------------------
# 离线运行器：无 ROS，直接跑数据集。评估基础设施优先级高于任何新特性。
# ---------------------------------------------------------------------------
add_executable(lvi_offline tools/offline_runner.cpp)
target_link_libraries(lvi_offline lvi_core)

# ---------------------------------------------------------------------------
# 按 ROS 版本接入：节点、bag 读取、安装规则
#   ROS1 → ros1/lvi_slam_node.cpp + rosbag（.bag）
#   ROS2 → ros2/lvi_slam_node.cpp + rosbag2（.db3/.mcap）
# 无 ROS 时只能读 KITTI 风格文件夹。
# ---------------------------------------------------------------------------
if(LVI_ROS EQUAL 1)
  target_compile_definitions(lvi_core PUBLIC LVI_ROS1)
  if(LVI_LIVOX_DRIVER STREQUAL "livox_ros_driver2")
    target_compile_definitions(lvi_core PUBLIC LVI_LIVOX_DRIVER2)
  endif()
  target_compile_definitions(lvi_core PRIVATE LVI_WITH_ROSBAG1)
  target_include_directories(lvi_core PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(lvi_core ${catkin_LIBRARIES})

  add_executable(lvi_slam_node ros1/lvi_slam_node.cpp)
  target_link_libraries(lvi_slam_node lvi_core)
  add_dependencies(lvi_core ${catkin_EXPORTED_TARGETS})

  install(TARGETS lvi_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
  install(TARGETS lvi_slam_node lvi_offline
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
  install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_core test/test_core.cpp)
    target_link_libraries(test_core lvi_core)
  endif()

elseif(LVI_ROS EQUAL 2)
  target_compile_definitions(lvi_core PUBLIC LVI_ROS2)
  add_executable(lvi_slam_node ros2/lvi_slam_node.cpp)
  target_link_libraries(lvi_slam_node lvi_core)
  ament_target_dependencies(lvi_slam_node
    rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros cv_bridge
    pcl_conversions livox_ros_driver2
  )

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

else()
  # 无 ROS：只有核心库 + 离线工具
  install(TARGETS lvi_core lvi_offline
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  install(DIRECTORY include/ DESTINATION include)

  include(CTest)
  if(BUILD_TESTING)
    find_package(GTest QUIET)
    if(GTest_FOUND AND "${GTEST_LIBRARIES};${GTest_DIR}" MATCHES "conda")
      message(FATAL_ERROR "lvi_slam: GTest 来自 conda（${GTest_DIR}${GTEST_LIBRARIES}），"
        "请先 conda deactivate 再重新配置（并清掉 build 目录）。")
    endif()
    if(GTest_FOUND)
      add_executable(test_core test/test_core.cpp)
      target_link_libraries(test_core lvi_core GTest::gtest_main)
      add_test(NAME test_core COMMAND test_core)
    endif()
  endif()
endif()
