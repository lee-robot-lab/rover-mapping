// ================================================================
// pose_conversions.cpp
// 설명: ORB의 T_c1w를 뒤집고 고정 T_c1base를 곱해 T_wbase를 계산한다.
// 사용법:
//   const auto T_wbase = cameraToBasePose(T_c1w, T_c1base);
// ================================================================

#include "orbslam3_ros2/pose_conversions.hpp"

namespace orbslam3_ros2
{

Sophus::SE3f cameraToBasePose(
  const Sophus::SE3f & T_c1w,
  const Sophus::SE3f & T_c1base)
{
  return T_c1w.inverse() * T_c1base;
}

}  // namespace orbslam3_ros2
