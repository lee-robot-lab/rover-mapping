// ================================================================
// test_pose_conversions.cpp
// 설명: ORB camera pose와 URDF camera-to-base 변환의 합성 순서를 검증한다.
// 사용법:
//   colcon test --packages-select orbslam3_ros2
// ================================================================

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "orbslam3_ros2/pose_conversions.hpp"

TEST(PoseConversions, CameraPoseUsesInverseTcwThenCameraToBase)
{
  const Sophus::SE3f T_c1w(
    Eigen::Matrix3f::Identity(), Eigen::Vector3f(-2.0F, 0.0F, 0.0F));
  const Sophus::SE3f T_c1base(
    Eigen::Matrix3f::Identity(), Eigen::Vector3f(-0.1F, 0.0F, 0.0F));

  const Sophus::SE3f T_wbase =
    orbslam3_ros2::cameraToBasePose(T_c1w, T_c1base);

  EXPECT_NEAR(T_wbase.translation().x(), 1.9F, 1e-6F);
  EXPECT_NEAR(T_wbase.translation().y(), 0.0F, 1e-6F);
  EXPECT_NEAR(T_wbase.translation().z(), 0.0F, 1e-6F);
}

TEST(PoseConversions, CameraToBaseTranslationIsRotatedIntoWorld)
{
  constexpr float kHalfPi = 1.57079632679F;
  const Eigen::AngleAxisf yaw(kHalfPi, Eigen::Vector3f::UnitZ());
  const Sophus::SE3f T_c1w(yaw.inverse().toRotationMatrix(), Eigen::Vector3f::Zero());
  const Sophus::SE3f T_c1base(
    Eigen::Matrix3f::Identity(), Eigen::Vector3f(1.0F, 0.0F, 0.0F));

  const Sophus::SE3f T_wbase =
    orbslam3_ros2::cameraToBasePose(T_c1w, T_c1base);

  EXPECT_NEAR(T_wbase.translation().x(), 0.0F, 1e-6F);
  EXPECT_NEAR(T_wbase.translation().y(), 1.0F, 1e-6F);
  EXPECT_NEAR(T_wbase.translation().z(), 0.0F, 1e-6F);
}
