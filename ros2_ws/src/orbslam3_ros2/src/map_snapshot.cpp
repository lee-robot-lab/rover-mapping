// ================================================================
// map_snapshot.cpp
// 설명: tracking 대응점으로 지도 소유권을 확인하고 짧은 lock 안에서 값만 복사한다.
// 사용법: captureMapSnapshot(system, source_stamp)
// ================================================================
#include "orbslam3_ros2/map_snapshot.hpp"
#include <cmath>
#include <mutex>
#include "System.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
namespace orbslam3_ros2 {
ORB_SLAM3::Map * identifyTrackedMap(ORB_SLAM3::System & system) {
  ORB_SLAM3::Map * selected = nullptr;
  for (auto * point : system.GetTrackedMapPoints()) {
    if (!point || point->isBad()) continue;
    auto * owner = point->GetMap();
    if (!owner || owner->IsBad()) return nullptr;
    if (selected && selected != owner) return nullptr;
    selected = owner;
  }
  return selected;
}
std::optional<MapSnapshot> captureMapValues(ORB_SLAM3::Map * map, double stamp) {
  if (!map || map->IsBad() || !std::isfinite(stamp)) return std::nullopt;
  std::unique_lock<std::mutex> lock(map->mMutexMapUpdate, std::try_to_lock);
  if (!lock.owns_lock()) return std::nullopt;
  MapSnapshot out{map->GetInitKFid(), map->GetMapChangeIndex(), stamp, {}, {}};
  for (auto * point : map->GetAllMapPoints()) {
    if (!point || point->isBad()) continue;
    if (point->GetMap() != map) return std::nullopt;
    auto position = point->GetWorldPos();
    if (position.allFinite()) out.points.push_back({point->mnId, position});
  }
  for (auto * frame : map->GetAllKeyFrames()) {
    if (!frame || frame->isBad()) continue;
    if (frame->GetMap() != map) return std::nullopt;
    auto pose = frame->GetPoseInverse();
    if (pose.matrix().allFinite() && std::isfinite(frame->mTimeStamp))
      out.keyframes.push_back({frame->mnId, frame->mTimeStamp, pose});
  }
  if (map->IsBad() || map->GetInitKFid() != out.map_id || map->GetMapChangeIndex() != out.change_index)
    return std::nullopt;
  return out;
}
std::optional<MapSnapshot> captureMapSnapshot(ORB_SLAM3::System & system, double stamp) {
  return captureMapValues(identifyTrackedMap(system), stamp);
}
}
