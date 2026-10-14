// ================================================================
// trajectory_io.hpp
// 설명: ROS PoseStamped sequence를 timestamp가 보존된 TUM trajectory로 저장한다.
// 사용법:
//   orbslam3_ros2::writeTumPoses(path, poses);
// ================================================================

#pragma once

#include <filesystem>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace orbslam3_ros2
{

void writeTumPoses(
  const std::filesystem::path & path,
  const std::vector<geometry_msgs::msg::PoseStamped> & poses);

}  // namespace orbslam3_ros2
