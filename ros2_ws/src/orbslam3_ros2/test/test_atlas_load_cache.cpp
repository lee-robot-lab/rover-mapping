// ================================================================
// test_atlas_load_cache.cpp
// 설명: Atlas load용 MapPoint 생성자가 재사용된 메모리의 tracking cache를 초기화하는지 검사한다.
// 사용법: colcon test --packages-select orbslam3_ros2
// ================================================================
#include <cstring>
#include <new>
#include <gtest/gtest.h>
#include "MapPoint.h"

TEST(AtlasLoadCache, InitializesTrackingStateOnReusedMemory) {
  for (const unsigned char pattern : {0x55, 0xff, 0x00}) {
    alignas(ORB_SLAM3::MapPoint) unsigned char storage[sizeof(ORB_SLAM3::MapPoint)];
    std::memset(storage, pattern, sizeof storage);
    auto * point = new(storage) ORB_SLAM3::MapPoint();
    unsigned char left, right;
    int left_level, right_level;
    // Inspect object bytes: reading an uninitialized bool directly would itself be UB.
    std::memcpy(&left, &point->mbTrackInView, 1);
    std::memcpy(&right, &point->mbTrackInViewR, 1);
    std::memcpy(&left_level, &point->mnTrackScaleLevel, sizeof left_level);
    std::memcpy(&right_level, &point->mnTrackScaleLevelR, sizeof right_level);
    EXPECT_EQ(left, 0) << "heap pattern=" << int(pattern);
    EXPECT_EQ(right, 0) << "heap pattern=" << int(pattern);
    EXPECT_EQ(left_level, 0);
    EXPECT_EQ(right_level, 0);
    point->~MapPoint();
  }
}
