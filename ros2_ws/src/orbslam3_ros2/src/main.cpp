// ================================================================
// main.cpp
// 설명: ORB-SLAM3 Stereo-Inertial ROS 2 node를 실행하고 clean shutdown을 보장한다.
// 사용법:
//   ros2 run orbslam3_ros2 orbslam3_stereo_inertial --ros-args ...
// ================================================================

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "orbslam3_ros2/stereo_inertial_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<orbslam3_ros2::StereoInertialNode>();
    rclcpp::spin(node);
    node->shutdownSlam();
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("orbslam3_stereo_inertial"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
