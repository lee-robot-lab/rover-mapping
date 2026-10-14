// ================================================================
// pose_conversions.hpp
// 설명: ORB camera-to-world 결과를 world-to-base pose로 합성한다.
// 사용법:
//   const auto T_wbase = cameraToBasePose(T_c1w, T_c1base);
// ================================================================

#pragma once

#include <sophus/se3.hpp>

namespace orbslam3_ros2
{

Sophus::SE3f cameraToBasePose(
  const Sophus::SE3f & T_c1w,
  const Sophus::SE3f & T_c1base);

}  // namespace orbslam3_ros2
