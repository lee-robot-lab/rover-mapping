// ================================================================
// test_sensor_mode.cpp
// 설명: stereo 단독 모드가 IMU를 요구하지 않으며 잘못된 모드를 거부하는지 검증한다.
// 사용법: colcon test --packages-select orbslam3_ros2
// ================================================================
#include <gtest/gtest.h>
#include <stdexcept>
#include "orbslam3_ros2/sensor_mode.hpp"
using namespace orbslam3_ros2;
TEST(SensorMode, IndependentStereo) { EXPECT_FALSE(requiresImu(parseSensorMode("stereo"))); }
TEST(SensorMode, InertialNeedsImu) { EXPECT_TRUE(requiresImu(parseSensorMode("stereo_inertial"))); }
TEST(SensorMode, InvalidMode) { EXPECT_THROW(parseSensorMode("rgbd"), std::invalid_argument); }
