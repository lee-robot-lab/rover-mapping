# ================================================================
# CMakeLists.txt
# 설명: 기존 native ORB-SLAM3로 같은 offline stereo 입력을 비교하는 실행기를 빌드한다.
# 사용법: cmake -S experiments/handheld_mapping/orbline/baseline -B outputs/handheld_mapping/orb3_comparison_build -DORB3_ROOT=...
# ================================================================
cmake_minimum_required(VERSION 3.16)
project(rover_orb3_comparison LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
add_executable(rover_orb3_compare ../runner.cpp)
target_compile_definitions(rover_orb3_compare PRIVATE ROVER_POINT_BASELINE EIGEN_HAS_CXX17_OVERALIGN=0)
target_compile_options(rover_orb3_compare PRIVATE -O1 -g0 -march=native)
target_include_directories(rover_orb3_compare PRIVATE ${ORB3_ROOT}/include ${ORB3_ROOT}/include/CameraModels
  ${ORB3_ROOT} ${ORB3_ROOT}/Thirdparty/Sophus ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
target_link_libraries(rover_orb3_compare ${ORB3_ROOT}/lib/libORB_SLAM3.so ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
