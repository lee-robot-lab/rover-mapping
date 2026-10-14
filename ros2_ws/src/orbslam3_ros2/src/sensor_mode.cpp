// ================================================================
// sensor_mode.cpp
// 설명: 명시된 두 센서 모드만 허용한다.
// 사용법: parseSensorMode(value)
// ================================================================
#include "orbslam3_ros2/sensor_mode.hpp"
#include <stdexcept>
namespace orbslam3_ros2 {
SensorMode parseSensorMode(const std::string & value) {
  if (value == "stereo") return SensorMode::Stereo;
  if (value == "stereo_inertial") return SensorMode::StereoInertial;
  throw std::invalid_argument("sensor_mode must be stereo or stereo_inertial");
}
bool requiresImu(SensorMode mode) { return mode == SensorMode::StereoInertial; }
}
