# ================================================================
# CMakeLists.txt
# 설명: 공식 ORB-SLAM3 Stereo-Inertial ROS 2 adapter와 unit test를 빌드한다.
# 사용법:
#   colcon build --packages-select orbslam3_ros2
# ================================================================

cmake_minimum_required(VERSION 3.16)
project(orbslam3_ros2)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Pangolin REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(ORB_SLAM3_ROOT "/opt/ORB_SLAM3" CACHE PATH "ORB-SLAM3 source root")
file(READ "${ORB_SLAM3_ROOT}/src/System.cc" ORB_SYSTEM_SOURCE)
string(FIND "${ORB_SYSTEM_SOURCE}" "ORB workers did not finish; Atlas not saved" SHUTDOWN_WAIT_MARKER)
if(SHUTDOWN_WAIT_MARKER EQUAL -1)
  message(FATAL_ERROR "Apply patches/orbslam3_shutdown_wait.patch and rebuild ORB_SLAM3 before this adapter")
endif()
file(READ "${ORB_SLAM3_ROOT}/src/MapPoint.cc" ORB_MAPPOINT_SOURCE)
string(FIND "${ORB_MAPPOINT_SOURCE}" "orbslam3_has_initialized_atlas_tracking_cache" ATLAS_CACHE_MARKER)
if(ATLAS_CACHE_MARKER EQUAL -1)
  message(FATAL_ERROR "Apply patches/orbslam3_atlas_load_cache.patch and rebuild ORB_SLAM3 before this adapter")
endif()
# Official ORB and bundled g2o use -march=native. Eigen allocation ABI must match.
set(ORB_ARCH_FLAGS "-march=native" CACHE STRING "Architecture flags used for ORB and g2o")
separate_arguments(ORB_ARCH_OPTIONS NATIVE_COMMAND "${ORB_ARCH_FLAGS}")
add_compile_options(${ORB_ARCH_OPTIONS})
# ORB/g2o are C++14. Preserve their Eigen class new/delete declarations in C++17 headers.
add_compile_definitions(EIGEN_HAS_CXX17_OVERALIGN=0)
set(ORB_SLAM3_INCLUDE_DIRS
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus)

add_executable(orbslam3_stereo_inertial
  src/main.cpp
  src/sensor_mode.cpp
  src/map_snapshot.cpp
  src/map_output.cpp
  src/imu_buffer.cpp
  src/pose_conversions.cpp
  src/stereo_inertial_node.cpp
  src/trajectory_io.cpp)
target_compile_features(orbslam3_stereo_inertial PRIVATE cxx_std_17)
target_compile_options(orbslam3_stereo_inertial PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(orbslam3_stereo_inertial PRIVATE
  include
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS})
ament_target_dependencies(orbslam3_stereo_inertial
  cv_bridge
  diagnostic_msgs
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs)
target_link_libraries(orbslam3_stereo_inertial
  Eigen3::Eigen
  OpenSSL::Crypto
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so)

set_target_properties(orbslam3_stereo_inertial PROPERTIES
  INSTALL_RPATH "${ORB_SLAM3_ROOT}/lib;${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib;${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib")
install(TARGETS orbslam3_stereo_inertial
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch config patches
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/topic_count_probe.py
  scripts/prepare_mapping_run.py scripts/mapping_probe.py scripts/record_mapping_run.py
  scripts/ksrc_telemetry_capture.py scripts/replay_stereo_sequence.py scripts/telemetry_tee.py scripts/finalize_mapping_run.py
  scripts/mapping_preflight.py scripts/mapping_session.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_atlas_load_cache test/test_atlas_load_cache.cpp)
  target_compile_features(test_atlas_load_cache PRIVATE cxx_std_17)
  target_include_directories(test_atlas_load_cache PRIVATE ${ORB_SLAM3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
  target_link_libraries(test_atlas_load_cache Eigen3::Eigen ${OpenCV_LIBS} ${Pangolin_LIBRARIES}
    ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so)

  ament_add_gtest(test_sensor_mode test/test_sensor_mode.cpp src/sensor_mode.cpp)
  target_include_directories(test_sensor_mode PRIVATE include)
  target_compile_features(test_sensor_mode PRIVATE cxx_std_17)
  ament_add_gtest(test_map_output test/test_map_output.cpp src/map_output.cpp)
  target_include_directories(test_map_output PRIVATE include ${ORB_SLAM3_ROOT}/Thirdparty/Sophus)
  target_compile_features(test_map_output PRIVATE cxx_std_17)
  target_link_libraries(test_map_output Eigen3::Eigen)
  ament_target_dependencies(test_map_output geometry_msgs sensor_msgs rclcpp)

  ament_add_gtest(test_imu_buffer
    test/test_imu_buffer.cpp
    src/imu_buffer.cpp)
  target_include_directories(test_imu_buffer PRIVATE include)
  target_compile_features(test_imu_buffer PRIVATE cxx_std_17)

  ament_add_gtest(test_pose_conversions
    test/test_pose_conversions.cpp
    src/pose_conversions.cpp)
  target_include_directories(test_pose_conversions PRIVATE
    include
    ${ORB_SLAM3_ROOT}/Thirdparty/Sophus)
  target_compile_features(test_pose_conversions PRIVATE cxx_std_17)
  target_link_libraries(test_pose_conversions Eigen3::Eigen)

  ament_add_gtest(test_trajectory_io
    test/test_trajectory_io.cpp
    src/trajectory_io.cpp)
  target_include_directories(test_trajectory_io PRIVATE include)
  target_compile_features(test_trajectory_io PRIVATE cxx_std_17)
  ament_target_dependencies(test_trajectory_io geometry_msgs)
endif()

ament_package()
