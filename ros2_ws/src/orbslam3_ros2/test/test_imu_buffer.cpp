// ================================================================
// test_imu_buffer.cpp
// 설명: stereo frame 구간별 IMU sample 소비와 timestamp 거부 규칙을 검증한다.
// 사용법:
//   colcon test --packages-select orbslam3_ros2
// ================================================================

#include <gtest/gtest.h>

#include <stdexcept>

#include "orbslam3_ros2/imu_buffer.hpp"

namespace
{

orbslam3_ros2::ImuSample sample(const double timestamp)
{
  return orbslam3_ros2::ImuSample{
    timestamp,
    {0.0F, 0.0F, 9.81F},
    {0.0F, 0.0F, 0.0F},
  };
}

}  // namespace

TEST(ImuBuffer, ReturnsOnlySamplesInImageInterval)
{
  orbslam3_ros2::ImuBuffer buffer;
  buffer.push(sample(0.00));
  buffer.push(sample(0.01));
  buffer.push(sample(0.02));

  const auto values = buffer.takeBetween(0.00, 0.02);

  ASSERT_EQ(values.size(), 2U);
  EXPECT_DOUBLE_EQ(values.front().t, 0.01);
  EXPECT_DOUBLE_EQ(values.back().t, 0.02);
}

TEST(ImuBuffer, RejectsDuplicateOrBackwardTimestamp)
{
  orbslam3_ros2::ImuBuffer buffer;
  buffer.push(sample(0.01));

  EXPECT_THROW(buffer.push(sample(0.01)), std::invalid_argument);
  EXPECT_THROW(buffer.push(sample(0.00)), std::invalid_argument);
}

TEST(ImuBuffer, EmptyIntervalIsReportedInsteadOfForwardingEmptyVector)
{
  orbslam3_ros2::ImuBuffer buffer;
  buffer.push(sample(0.00));

  EXPECT_THROW(buffer.takeBetween(0.00, 0.03), std::runtime_error);
}
