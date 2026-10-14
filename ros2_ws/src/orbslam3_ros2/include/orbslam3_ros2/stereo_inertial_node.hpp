// ================================================================
// stereo_inertial_node.hpp
// 설명: 동기 stereo image와 IMU를 공식 ORB-SLAM3 Stereo-Inertial API에 연결한다.
// 사용법:
//   ros2 run orbslam3_ros2 orbslam3_stereo_inertial --ros-args ...
// ================================================================

#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "orbslam3_ros2/sensor_mode.hpp"
#include "orbslam3_ros2/map_snapshot.hpp"

#include <sophus/se3.hpp>
#include <opencv2/core.hpp>

#include "orbslam3_ros2/imu_buffer.hpp"

namespace ORB_SLAM3
{
class System;
}  // namespace ORB_SLAM3

namespace orbslam3_ros2
{

class StereoInertialNode : public rclcpp::Node
{
  /// ROS 2 stereo/IMU stream을 ORB-SLAM3에 전달하고 camera/base trajectory를 발행한다.
public:
  StereoInertialNode();
  ~StereoInertialNode() override;

  void shutdownSlam();

private:
  using Image = sensor_msgs::msg::Image;
  using ExactStereoPolicy = message_filters::sync_policies::ExactTime<Image, Image>;

  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr message);
  void onStereo(
    const Image::ConstSharedPtr & left,
    const Image::ConstSharedPtr & right);
  void publishPosePair(
    const std_msgs::msg::Header & header,
    const Sophus::SE3f & T_wc1,
    const Sophus::SE3f & T_wbase);
  void publishTrackingState(int state);
  void writeTrackingReport() const;
  void publishDiagnostics();
  void saveAlignments() const;
  void loadAlignments();

  std::unique_ptr<ORB_SLAM3::System> slam_;
  Sophus::SE3f T_c1base_;
  SensorMode sensor_mode_{SensorMode::StereoInertial};
  std::string preprocessing_, run_status_{"running"};
  int width_{0}, height_{0}, state_{-1};
  bool overlay_coordinates_valid_{false};
  bool publish_base_{true};
  cv::Mat overlay_map_x_, overlay_map_y_;
  std::optional<std::uint64_t> current_map_, last_valid_map_;
  std::map<std::uint64_t, Sophus::SE3f> alignments_;
  std::map<std::uint64_t, nav_msgs::msg::Path> histories_;
  std::optional<MapSnapshot> latest_snapshot_;
  double snapshot_stamp_{-1e30}, last_pose_stamp_{0}, process_ms_{0}, snapshot_ms_{0};
  std::uint64_t snapshot_skips_{0}, invalid_map_drops_{0};
  std::optional<std::uint64_t> orb_runtime_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr keyframes_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<Image>::SharedPtr overlay_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::filesystem::path trajectory_dir_;

  message_filters::Subscriber<Image> left_subscriber_;
  message_filters::Subscriber<Image> right_subscriber_;
  std::shared_ptr<message_filters::Synchronizer<ExactStereoPolicy>> stereo_sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr base_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tracking_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

  mutable std::mutex imu_mutex_;
  ImuBuffer imu_buffer_;
  std::optional<double> last_image_stamp_;
  double last_path_publish_stamp_{-1e30};
  std::optional<std::uint64_t> path_published_map_;
  bool shutdown_complete_{false};
  bool require_prepared_run_{false};
  bool atlas_load_verified_{false};

  std::uint64_t image_pairs_{0};
  std::uint64_t accepted_imu_samples_{0};
  std::uint64_t rejected_imu_samples_{0};
  std::uint64_t empty_imu_interval_drops_{0};
  std::uint64_t conversion_failures_{0};
  std::uint64_t tracking_failures_{0};
  std::uint64_t published_poses_{0};
  std::map<int, std::uint64_t> tracking_state_counts_;
};

}  // namespace orbslam3_ros2
