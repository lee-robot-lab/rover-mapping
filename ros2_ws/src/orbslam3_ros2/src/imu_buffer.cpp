// ================================================================
// imu_buffer.cpp
// 설명: IMU timestamp 단조성 검증과 frame 구간별 destructive consume을 구현한다.
// 사용법:
//   orbslam3_ros2::ImuBuffer buffer;
// ================================================================

#include "orbslam3_ros2/imu_buffer.hpp"

#include <cmath>
#include <stdexcept>

namespace orbslam3_ros2
{

void ImuBuffer::push(const ImuSample & sample)
{
  const auto finite_vector = [](const std::array<float, 3> & values) {
      return std::isfinite(values[0]) && std::isfinite(values[1]) &&
             std::isfinite(values[2]);
    };
  if (!std::isfinite(sample.t) || !finite_vector(sample.acceleration) ||
    !finite_vector(sample.angular_velocity))
  {
    throw std::invalid_argument("IMU sample must contain finite values");
  }
  if (last_pushed_t_.has_value() && sample.t <= *last_pushed_t_) {
    throw std::invalid_argument("IMU timestamps must be strictly increasing");
  }
  samples_.push_back(sample);
  last_pushed_t_ = sample.t;
}

std::vector<ImuSample> ImuBuffer::takeBetween(
  const double previous_image_t, const double current_image_t)
{
  if (!std::isfinite(previous_image_t) || !std::isfinite(current_image_t) ||
    current_image_t <= previous_image_t)
  {
    throw std::invalid_argument("image interval must be finite and increasing");
  }

  while (!samples_.empty() && samples_.front().t <= previous_image_t) {
    samples_.pop_front();
  }

  std::vector<ImuSample> selected;
  while (!samples_.empty() && samples_.front().t <= current_image_t) {
    selected.push_back(samples_.front());
    samples_.pop_front();
  }
  if (selected.empty()) {
    throw std::runtime_error("no IMU samples in image interval");
  }
  return selected;
}

std::size_t ImuBuffer::size() const noexcept
{
  return samples_.size();
}

}  // namespace orbslam3_ros2
