// ================================================================
// sensor_mode.hpp
// 설명: 공식 ORB stereo와 stereo-inertial 모드의 입력 계약을 정의한다.
// 사용법: parseSensorMode("stereo")
// ================================================================
#pragma once
#include <string>
namespace orbslam3_ros2 {
enum class SensorMode { Stereo, StereoInertial };
SensorMode parseSensorMode(const std::string & value);
bool requiresImu(SensorMode mode);
}
