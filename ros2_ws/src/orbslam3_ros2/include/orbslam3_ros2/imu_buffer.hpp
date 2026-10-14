// ================================================================
// imu_buffer.hpp
// 설명: 단조 증가하는 IMU sample을 stereo frame 시간 구간별로 소비한다.
// 사용법:
//   orbslam3_ros2::ImuBuffer buffer;
// ================================================================

#pragma once

#include <array>
#include <deque>
#include <optional>
#include <vector>

namespace orbslam3_ros2
{

struct ImuSample
{
  /// ORB-SLAM3에 전달할 한 IMU 측정값과 simulation timestamp를 보관한다.
  double t;
  std::array<float, 3> acceleration;
  std::array<float, 3> angular_velocity;
};

class ImuBuffer
{
  /// 엄격히 증가하는 IMU stream을 보관하고 image interval별로 한 번만 소비한다.
public:
  void push(const ImuSample & sample);
  std::vector<ImuSample> takeBetween(double previous_image_t, double current_image_t);
  std::size_t size() const noexcept;

private:
  std::deque<ImuSample> samples_;
  std::optional<double> last_pushed_t_;
};

}  // namespace orbslam3_ros2
