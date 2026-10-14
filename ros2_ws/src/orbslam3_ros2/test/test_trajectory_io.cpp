// ================================================================
// test_trajectory_io.cpp
// 설명: ROS PoseStamped base trajectory가 원자적 TUM 파일로 저장되는지 검증한다.
// 사용법:
//   colcon test --packages-select orbslam3_ros2
// ================================================================

#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "orbslam3_ros2/trajectory_io.hpp"

namespace
{

geometry_msgs::msg::PoseStamped pose(const int seconds, const int nanoseconds, const double x)
{
  geometry_msgs::msg::PoseStamped value;
  value.header.stamp.sec = seconds;
  value.header.stamp.nanosec = nanoseconds;
  value.pose.position.x = x;
  value.pose.position.y = 0.2;
  value.pose.position.z = 0.3;
  value.pose.orientation.w = 1.0;
  return value;
}

}  // namespace

TEST(TrajectoryIo, WritesParseableTumRowsWithoutLeavingTemporaryFile)
{
  const std::filesystem::path root =
    std::filesystem::temp_directory_path() / "orbslam3_ros2_trajectory_io_test";
  std::filesystem::create_directories(root);
  const std::filesystem::path output = root / "base_trajectory.tum";
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pose(1, 100000000, 0.0),
    pose(1, 200000000, 0.1),
  };

  orbslam3_ros2::writeTumPoses(output, poses);

  std::ifstream stream(output);
  const std::string contents{
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  EXPECT_NE(contents.find("1.100000000 0.000000000 0.200000000 0.300000000"), std::string::npos);
  EXPECT_NE(contents.find("1.200000000 0.100000000 0.200000000 0.300000000"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(root / "base_trajectory.tum.tmp"));

  std::filesystem::remove(output);
  std::filesystem::remove(root);
}

TEST(TrajectoryIo, RejectsNonIncreasingTimestamps)
{
  const std::vector<geometry_msgs::msg::PoseStamped> poses{
    pose(1, 100000000, 0.0),
    pose(1, 100000000, 0.1),
  };

  EXPECT_THROW(
    orbslam3_ros2::writeTumPoses("/tmp/invalid_base_trajectory.tum", poses),
    std::invalid_argument);
}
