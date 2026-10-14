// ================================================================
// trajectory_io.cpp
// 설명: 유한·단조 ROS pose를 검증하고 TUM 파일을 임시 파일에서 원자적으로 공개한다.
// 사용법:
//   orbslam3_ros2::writeTumPoses(path, poses);
// ================================================================

#include "orbslam3_ros2/trajectory_io.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace orbslam3_ros2
{

void writeTumPoses(
  const std::filesystem::path & path,
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  if (poses.empty()) {
    throw std::invalid_argument("TUM trajectory must not be empty");
  }
  double previous_stamp = -1.0;
  for (const auto & stamped : poses) {
    const double stamp = static_cast<double>(stamped.header.stamp.sec) +
      static_cast<double>(stamped.header.stamp.nanosec) * 1.0e-9;
    const auto & position = stamped.pose.position;
    const auto & orientation = stamped.pose.orientation;
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    if (!std::isfinite(stamp) || stamp <= previous_stamp ||
      !std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(quaternion_norm) ||
      quaternion_norm < 1.0e-12)
    {
      throw std::invalid_argument("TUM poses must be finite with increasing timestamps");
    }
    previous_stamp = stamp;
  }

  const std::filesystem::path temporary = path.string() + ".tmp";
  std::ofstream stream(temporary);
  if (!stream) {
    throw std::runtime_error("failed to open TUM trajectory temporary file");
  }
  stream << std::fixed << std::setprecision(9);
  for (const auto & stamped : poses) {
    const double stamp = static_cast<double>(stamped.header.stamp.sec) +
      static_cast<double>(stamped.header.stamp.nanosec) * 1.0e-9;
    const auto & position = stamped.pose.position;
    const auto & orientation = stamped.pose.orientation;
    const double quaternion_norm = std::sqrt(
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w);
    stream << stamp << ' '
           << position.x << ' ' << position.y << ' ' << position.z << ' '
           << orientation.x / quaternion_norm << ' '
           << orientation.y / quaternion_norm << ' '
           << orientation.z / quaternion_norm << ' '
           << orientation.w / quaternion_norm << '\n';
  }
  stream.close();
  if (!stream) {
    throw std::runtime_error("failed to write TUM trajectory");
  }
  std::filesystem::rename(temporary, path);
}

}  // namespace orbslam3_ros2
