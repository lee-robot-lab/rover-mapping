// ================================================================
// map_snapshot.hpp
// 설명: ORB 객체를 callback 밖에 보관하지 않고 지도 값을 복사한다.
// 사용법: captureMapSnapshot(system, source_stamp)
// ================================================================
#pragma once
#include <cstdint>
#include <optional>
#include <vector>
#include <Eigen/Core>
#include <sophus/se3.hpp>
namespace ORB_SLAM3 { class System; class Map; }
namespace orbslam3_ros2 {
struct MapPointValue { std::uint64_t id; Eigen::Vector3f position; };
struct KeyFrameValue { std::uint64_t id; double stamp; Sophus::SE3f T_wc1; };
struct MapSnapshot {
  std::uint64_t map_id;
  int change_index;
  double source_stamp;
  std::vector<MapPointValue> points;
  std::vector<KeyFrameValue> keyframes;
};
ORB_SLAM3::Map * identifyTrackedMap(ORB_SLAM3::System & system);
std::optional<MapSnapshot> captureMapSnapshot(ORB_SLAM3::System & system, double stamp);
std::optional<MapSnapshot> captureMapValues(ORB_SLAM3::Map * map, double stamp);
}
