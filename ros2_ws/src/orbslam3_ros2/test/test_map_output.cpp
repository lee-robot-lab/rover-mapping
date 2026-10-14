// ================================================================
// test_map_output.cpp
// 설명: 지도 점과 키프레임에 동일한 변환을 쓰고 비유한 점은 버리는지 검증한다.
// 사용법: colcon test --packages-select orbslam3_ros2
// ================================================================
#include <gtest/gtest.h>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "orbslam3_ros2/map_output.hpp"
using namespace orbslam3_ros2;
TEST(MapOutput, ConsistentCoordinates) {
  MapSnapshot s{7, 0, 1., {{1, Eigen::Vector3f(1,0,0)}}, {{2,1.,Sophus::SE3f()}}};
  Sophus::SE3f t(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0,2,0));
  auto cloud = makeMapCloud(s,t); auto poses = makeKeyframePoses(s,t);
  EXPECT_EQ(cloud.header.frame_id, "orb_map_7"); EXPECT_EQ(poses.header.frame_id,cloud.header.frame_id);
  EXPECT_NEAR(poses.poses.at(0).position.y,2.,1e-6);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud,"x"),y(cloud,"y");
  EXPECT_NEAR(*x,1.,1e-6); EXPECT_NEAR(*y,2.,1e-6);
}
TEST(MapOutput, RejectNonfinitePoints) {
  MapSnapshot s{8,0,2.,{{1,Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(),0,0)}},{}};
  EXPECT_EQ(makeMapCloud(s,Sophus::SE3f()).width,0U);
}
