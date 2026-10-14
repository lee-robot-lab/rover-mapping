// ================================================================
// map_output.hpp
// 설명: 같은 지도 정렬로 점군·최적화 키프레임·파일 출력을 만든다.
// 사용법: makeMapCloud(snapshot, T_map_orb)
// ================================================================
#pragma once
#include <filesystem>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "orbslam3_ros2/map_snapshot.hpp"
namespace orbslam3_ros2 {
sensor_msgs::msg::PointCloud2 makeMapCloud(const MapSnapshot &, const Sophus::SE3f &);
geometry_msgs::msg::PoseArray makeKeyframePoses(const MapSnapshot &, const Sophus::SE3f &);
void exportMap(const std::filesystem::path &, const MapSnapshot &, const Sophus::SE3f &);
}
