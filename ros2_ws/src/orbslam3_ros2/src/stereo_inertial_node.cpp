// ================================================================
// stereo_inertial_node.cpp
// 설명: ExactTime stereo, sensor QoS IMU, ORB 추적과 ROS pose/report 발행을 구현한다.
// 사용법:
//   ros2 run orbslam3_ros2 orbslam3_stereo_inertial --ros-args ...
// ================================================================

#include "orbslam3_ros2/stereo_inertial_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "Map.h"
#include "MapPoint.h"
#include "orbslam3_ros2/map_output.hpp"
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>

#include "System.h"
#include "Tracking.h"
#include "orbslam3_ros2/pose_conversions.hpp"
#include "orbslam3_ros2/trajectory_io.hpp"

extern "C" bool orbslam3_has_bounded_shutdown_wait();
extern "C" bool orbslam3_has_initialized_atlas_tracking_cache();

namespace
{

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1.0e-9;
}

Sophus::SE3f loadCameraToBase(const std::string & settings_path)
{
  const cv::FileStorage storage(settings_path, cv::FileStorage::READ);
  if (!storage.isOpened()) {
    throw std::runtime_error("failed to open ORB settings: " + settings_path);
  }
  cv::Mat matrix;
  storage["ROS.T_c1_base"] >> matrix;
  if (matrix.rows != 4 || matrix.cols != 4) {
    throw std::runtime_error("ROS.T_c1_base must be a 4x4 matrix");
  }
  cv::Mat matrix_float;
  matrix.convertTo(matrix_float, CV_32F);
  Eigen::Matrix3f rotation;
  Eigen::Vector3f translation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation(row, column) = matrix_float.at<float>(row, column);
    }
    translation(row) = matrix_float.at<float>(row, 3);
  }
  if (!rotation.allFinite() || !translation.allFinite() ||
    !(rotation.transpose()*rotation).isApprox(Eigen::Matrix3f::Identity(), 1e-4f) ||
    std::abs(rotation.determinant()-1.f) > 1e-4f ||
    std::abs(matrix_float.at<float>(3,3)-1.f) > 1e-5f ||
    cv::norm(matrix_float.row(3).colRange(0,3)) > 1e-5) {
    throw std::runtime_error("ROS.T_c1_base contains non-finite values");
  }
  return Sophus::SE3f(rotation, translation);
}

geometry_msgs::msg::Pose toRosPose(const Sophus::SE3f & transform)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Vector3f translation = transform.translation();
  const Eigen::Quaternionf quaternion = transform.unit_quaternion();
  pose.position.x = static_cast<double>(translation.x());
  pose.position.y = static_cast<double>(translation.y());
  pose.position.z = static_cast<double>(translation.z());
  pose.orientation.x = static_cast<double>(quaternion.x());
  pose.orientation.y = static_cast<double>(quaternion.y());
  pose.orientation.z = static_cast<double>(quaternion.z());
  pose.orientation.w = static_cast<double>(quaternion.w());
  return pose;
}

}  // namespace

namespace orbslam3_ros2
{

StereoInertialNode::StereoInertialNode()
: Node("orbslam3_stereo_inertial")
{
  if (!orbslam3_has_bounded_shutdown_wait()) throw std::runtime_error("unpatched ORB shutdown");
  if (!orbslam3_has_initialized_atlas_tracking_cache()) throw std::runtime_error("unpatched ORB Atlas tracking cache");
  const std::string vocabulary_path = declare_parameter<std::string>("vocabulary_path", "");
  const std::string settings_path = declare_parameter<std::string>("settings_path", "");
  const std::string trajectory_path = declare_parameter<std::string>("trajectory_dir", "");
  const std::string left_topic = declare_parameter<std::string>(
    "left_topic", "/rover/stereo/left/image_raw");
  const std::string right_topic = declare_parameter<std::string>(
    "right_topic", "/rover/stereo/right/image_raw");
  const std::string imu_topic = declare_parameter<std::string>(
    "imu_topic", "/rover/imu/data");
  if (vocabulary_path.empty() || settings_path.empty() || trajectory_path.empty()) {
    throw std::invalid_argument(
            "vocabulary_path, settings_path, and trajectory_dir parameters are required");
  }

  sensor_mode_ = parseSensorMode(declare_parameter<std::string>("sensor_mode", "stereo_inertial"));
  preprocessing_ = declare_parameter<std::string>("image_preprocessing", "raw");
  if (preprocessing_ != "raw" && preprocessing_ != "clahe")
    throw std::invalid_argument("image_preprocessing must be raw or clahe");
  if (!std::filesystem::is_regular_file(vocabulary_path))
    throw std::invalid_argument("vocabulary file missing");
  const cv::FileStorage settings(settings_path, cv::FileStorage::READ);
  if (!settings.isOpened()) throw std::invalid_argument("settings file missing");
  width_ = static_cast<int>(settings["Camera.width"]);
  height_ = static_cast<int>(settings["Camera.height"]);
  if (width_ <= 0 || height_ <= 0) throw std::invalid_argument("expected image dimensions missing");
  cv::Mat stereo_transform;
  settings["Stereo.T_c1_c2"] >> stereo_transform;
  if (stereo_transform.rows != 4 || stereo_transform.cols != 4)
    throw std::invalid_argument("Stereo.T_c1_c2 missing");
  trajectory_dir_ = std::filesystem::absolute(trajectory_path);
  std::filesystem::create_directories(trajectory_dir_);
  const std::string pose_output = settings["ROS.pose_output"].empty() ?
    "base" : static_cast<std::string>(settings["ROS.pose_output"]);
  if (pose_output != "base" && pose_output != "camera")
    throw std::invalid_argument("ROS.pose_output must be base or camera");
  publish_base_ = pose_output == "base";
  if (publish_base_) T_c1base_ = loadCameraToBase(settings_path);
  else if (!settings["ROS.T_c1_base"].empty())
    throw std::invalid_argument("camera output must not claim a robot base transform");
  if (static_cast<std::string>(settings["Camera.type"]) == "PinHole" &&
      static_cast<std::string>(settings["File.version"]) == "1.0") {
    // Match official Settings::precomputeRectificationMaps. TrackStereo poses use rectified c1.
    cv::Mat cameras[2], distortion[2];
    for(int i=0;i<2;++i) {
      const std::string prefix="Camera"+std::to_string(i+1)+".";
      const auto number=[&](const char * key) { return static_cast<double>(static_cast<float>(settings[prefix+key])); };
      cameras[i]=(cv::Mat_<double>(3,3)<<number("fx"),0.,number("cx"),0.,number("fy"),number("cy"),0.,0.,1.);
      distortion[i]=(cv::Mat_<float>(1,5)<<number("k1"),number("k2"),number("p1"),number("p2"),number("k3"));
    }
    cv::Mat tf; stereo_transform.convertTo(tf,CV_32F);
    cv::Mat rotation=tf(cv::Rect(0,0,3,3)), translation=tf(cv::Rect(3,0,1,3));
    cv::Mat inverse_rotation=rotation.t(), inverse_translation=-inverse_rotation*translation;
    inverse_rotation.convertTo(inverse_rotation,CV_64F); inverse_translation.convertTo(inverse_translation,CV_64F);
    cv::Mat r1,r2,p1,p2,q;
    cv::stereoRectify(cameras[0],distortion[0],cameras[1],distortion[1],cv::Size(width_,height_),
      inverse_rotation,inverse_translation,r1,r2,p1,p2,q,cv::CALIB_ZERO_DISPARITY,-1,cv::Size(width_,height_));
    cv::initUndistortRectifyMap(cameras[0],distortion[0],r1,p1(cv::Rect(0,0,3,3)),
      cv::Size(width_,height_),CV_32F,overlay_map_x_,overlay_map_y_);
    Eigen::Matrix3f rectification;
    for(int r=0;r<3;++r) for(int c=0;c<3;++c) rectification(r,c)=r1.at<double>(r,c);
    if (publish_base_) T_c1base_=Sophus::SE3f(rectification,Eigen::Vector3f::Zero())*T_c1base_;
    overlay_coordinates_valid_=true;
  }
  loadAlignments();
  require_prepared_run_ = declare_parameter<bool>("require_prepared_run", false);
  if (require_prepared_run_) {
    if (std::filesystem::current_path() != trajectory_dir_ ||
        !std::filesystem::is_regular_file(trajectory_dir_/"manifest.json"))
      throw std::invalid_argument("prepared run cwd and manifest required");
    const auto started=trajectory_dir_/"mapping.started";
    const int fd=::open(started.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
    if(fd<0) throw std::runtime_error("run already started or is not writable; prepare a new run");
    ::close(fd);
  }
  slam_ = std::make_unique<ORB_SLAM3::System>(
    vocabulary_path, settings_path, requiresImu(sensor_mode_) ? ORB_SLAM3::System::IMU_STEREO : ORB_SLAM3::System::STEREO, false);
  const auto load_node=settings["System.LoadAtlasFromFile"];
  atlas_load_verified_=!load_node.empty() && !static_cast<std::string>(load_node).empty();

  camera_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/orbslam3/camera_pose", 10);
  if (publish_base_) base_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "/orbslam3/base_pose", 10);
  tracking_state_publisher_ = create_publisher<std_msgs::msg::Int32>(
    "/orbslam3/tracking_state", 10);
  path_publisher_ = create_publisher<nav_msgs::msg::Path>("/orbslam3/path", 10);

  cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/orbslam3/map_points", 1);
  keyframes_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>("/orbslam3/keyframes", 1);
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/orbslam3/diagnostics", 10);
  overlay_publisher_ = create_publisher<Image>("/orbslam3/tracked_image", rclcpp::SensorDataQoS());
  diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&StereoInertialNode::publishDiagnostics, this));
  if (requiresImu(sensor_mode_)) imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    std::bind(&StereoInertialNode::onImu, this, std::placeholders::_1));
  left_subscriber_.subscribe(this, left_topic, rmw_qos_profile_sensor_data);
  right_subscriber_.subscribe(this, right_topic, rmw_qos_profile_sensor_data);
  stereo_sync_ = std::make_shared<message_filters::Synchronizer<ExactStereoPolicy>>(
    ExactStereoPolicy(20), left_subscriber_, right_subscriber_);
  stereo_sync_->registerCallback(
    std::bind(
      &StereoInertialNode::onStereo, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(), "waiting for exact stereo on %s and %s; mode=%s; IMU=%s",
    left_topic.c_str(), right_topic.c_str(), requiresImu(sensor_mode_) ? "stereo_inertial" : "stereo",
    requiresImu(sensor_mode_) ? imu_topic.c_str() : "disabled");
}

StereoInertialNode::~StereoInertialNode()
{
  try {
    shutdownSlam();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "ORB shutdown failed: %s", error.what());
  }
}

void StereoInertialNode::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  const ImuSample sample{
    stampSeconds(message->header.stamp),
    {
      static_cast<float>(message->linear_acceleration.x),
      static_cast<float>(message->linear_acceleration.y),
      static_cast<float>(message->linear_acceleration.z),
    },
    {
      static_cast<float>(message->angular_velocity.x),
      static_cast<float>(message->angular_velocity.y),
      static_cast<float>(message->angular_velocity.z),
    },
  };
  try {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.push(sample);
  } catch (const std::invalid_argument & error) {
    ++rejected_imu_samples_;
    RCLCPP_WARN(get_logger(), "rejected IMU sample: %s", error.what());
  }
}

void StereoInertialNode::onStereo(
  const Image::ConstSharedPtr & left,
  const Image::ConstSharedPtr & right)
{
  if (shutdown_complete_) return;
  const auto processing_start = std::chrono::steady_clock::now();
  ++image_pairs_;
  current_map_.reset();
  const double stamp = stampSeconds(left->header.stamp);
  if (left->header.stamp != right->header.stamp ||
    left->width != static_cast<unsigned>(width_) || left->height != static_cast<unsigned>(height_) ||
    right->width != left->width || right->height != left->height ||
    left->encoding != sensor_msgs::image_encodings::MONO8 || right->encoding != sensor_msgs::image_encodings::MONO8 ||
    !std::isfinite(stamp) ||
    (last_image_stamp_.has_value() && stamp <= *last_image_stamp_))
  {
    ++conversion_failures_;
    RCLCPP_WARN(get_logger(), "rejected stereo stamp/size/mono8 contract");
    return;
  }

  cv::Mat left_mono;
  cv::Mat right_mono;
  try {
    left_mono = cv_bridge::toCvCopy(
      left, sensor_msgs::image_encodings::MONO8)->image;
    right_mono = cv_bridge::toCvCopy(
      right, sensor_msgs::image_encodings::MONO8)->image;
  } catch (const cv_bridge::Exception & error) {
    ++conversion_failures_;
    last_image_stamp_ = stamp;
    RCLCPP_WARN(get_logger(), "image conversion failed: %s", error.what());
    return;
  }

  std::vector<ImuSample> samples;
  const double previous_stamp = last_image_stamp_.value_or(
    std::numeric_limits<double>::lowest());
  last_image_stamp_ = stamp;
  try {
    if (requiresImu(sensor_mode_)) {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      samples = imu_buffer_.takeBetween(previous_stamp, stamp);
    }
  } catch (const std::runtime_error &) {
    ++empty_imu_interval_drops_;
    return;
  }
  accepted_imu_samples_ += samples.size();

  std::vector<ORB_SLAM3::IMU::Point> imu_points;
  imu_points.reserve(samples.size());
  for (const ImuSample & sample : samples) {
    imu_points.emplace_back(
      sample.acceleration[0], sample.acceleration[1], sample.acceleration[2],
      sample.angular_velocity[0], sample.angular_velocity[1],
      sample.angular_velocity[2], sample.t);
  }

  if (preprocessing_ == "clahe") {
    auto clahe = cv::createCLAHE(3., cv::Size(8,8));
    clahe->apply(left_mono, left_mono); clahe->apply(right_mono, right_mono);
  }
  try {
    const Sophus::SE3f T_c1w = slam_->TrackStereo(
      left_mono, right_mono, stamp, imu_points);
    const int state = slam_->GetTrackingState();
    state_ = state;
    current_map_.reset();
    ++tracking_state_counts_[state];
    publishTrackingState(state);
    if (state == ORB_SLAM3::Tracking::OK && T_c1w.matrix().allFinite()) {
      const Sophus::SE3f T_wc1 = T_c1w.inverse();
      // Handheld aligns to the initial camera pose; no robot pose is inferred.
      const Sophus::SE3f T_wreference = publish_base_ ? cameraToBasePose(T_c1w, T_c1base_) : T_wc1;
      auto * map = identifyTrackedMap(*slam_);
      if (!map) { ++invalid_map_drops_; return; }
      const auto id = map->GetInitKFid();
      orb_runtime_map_ = map->GetId();
      current_map_ = id;
      last_valid_map_ = id;
      if (alignments_.count(id) == 0) {
        alignments_.emplace(id, T_wreference.inverse());
        saveAlignments();
      }
      const auto & alignment = alignments_.at(id);
      publishPosePair(left->header, alignment * T_wc1, alignment * T_wreference);
      if (stamp-snapshot_stamp_ >= 1.) {
        const auto copy_start = std::chrono::steady_clock::now();
        auto snapshot = captureMapSnapshot(*slam_, stamp);
        snapshot_ms_ = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-copy_start).count();
        if (snapshot && snapshot->map_id == id) {
          latest_snapshot_ = std::move(snapshot);
          cloud_publisher_->publish(makeMapCloud(*latest_snapshot_, alignment));
          keyframes_publisher_->publish(makeKeyframePoses(*latest_snapshot_, alignment));
          snapshot_stamp_ = stamp;
        } else ++snapshot_skips_;
      }
    }
    if (overlay_coordinates_valid_ && overlay_publisher_->get_subscription_count() > 0) {
      cv::Mat image, rectified;
      cv::remap(left_mono,rectified,overlay_map_x_,overlay_map_y_,cv::INTER_LINEAR);
      cv::cvtColor(rectified, image, cv::COLOR_GRAY2BGR);
      if (state == ORB_SLAM3::Tracking::OK) {
        auto keys = slam_->GetTrackedKeyPointsUn(); auto points = slam_->GetTrackedMapPoints();
        if (keys.size() == points.size()) {
          for (std::size_t i=0; i<keys.size(); ++i)
            if (points[i] && !points[i]->isBad()) cv::circle(image, keys[i].pt, 2, cv::Scalar(0,255,0), 1);
        }
      }
      overlay_publisher_->publish(*cv_bridge::CvImage(left->header, "bgr8", image).toImageMsg());
    }
    process_ms_ = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-processing_start).count();
  } catch (const std::exception & error) {
    current_map_.reset();
    ++tracking_failures_;
    RCLCPP_ERROR(get_logger(), "TrackStereo failed: %s", error.what());
  }
}

void StereoInertialNode::publishPosePair(
  const std_msgs::msg::Header & header,
  const Sophus::SE3f & T_wc1,
  const Sophus::SE3f & T_wbase)
{
  geometry_msgs::msg::PoseStamped camera_pose;
  camera_pose.header = header;
  camera_pose.header.frame_id = "orb_map_" + std::to_string(*current_map_);
  camera_pose.pose = toRosPose(T_wc1);
  camera_pose_publisher_->publish(camera_pose);

  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.header = camera_pose.header;
  base_pose.pose = toRosPose(T_wbase);
  if (publish_base_) base_pose_publisher_->publish(base_pose);

  auto & history = histories_[*current_map_];
  const auto & path_pose = publish_base_ ? base_pose : camera_pose;
  history.header = path_pose.header;
  history.poses.push_back(path_pose);
  const double source_stamp=stampSeconds(header.stamp);
  if (path_published_map_ != current_map_ || source_stamp-last_path_publish_stamp_ >= .2) {
    path_publisher_->publish(history); last_path_publish_stamp_=source_stamp; path_published_map_=current_map_;
  }
  last_pose_stamp_ = stampSeconds(header.stamp);
  ++published_poses_;
}

void StereoInertialNode::publishTrackingState(const int state)
{
  std_msgs::msg::Int32 message;
  message.data = state;
  tracking_state_publisher_->publish(message);
}

void StereoInertialNode::shutdownSlam()
{
  if (shutdown_complete_) {
    return;
  }
  shutdown_complete_ = true;
  try {
    if (slam_) {
      auto * final_map = identifyTrackedMap(*slam_);  // 종료 범위에서만 유지하며 worker 종료 후 재확인한다.
      const auto expected_id = final_map ? std::optional<std::uint64_t>(final_map->GetInitKFid()) : std::nullopt;
      slam_->Shutdown();
      bool final_exported=false;
      auto final_snapshot = captureMapValues(final_map, last_image_stamp_.value_or(0.));
      if (final_snapshot && expected_id == final_snapshot->map_id && alignments_.count(final_snapshot->map_id)) {
        latest_snapshot_ = std::move(final_snapshot);
        exportMap(trajectory_dir_, *latest_snapshot_, alignments_.at(latest_snapshot_->map_id));
        final_exported=true;
      }
      for (const auto & item : histories_)
        writeTumPoses(trajectory_dir_ / ((publish_base_ ? "base_map_" : "camera_map_")+std::to_string(item.first)+".tum"), item.second.poses);
      saveAlignments();
      const auto atlas = trajectory_dir_ / "atlas.osa";
      const bool atlas_saved = std::filesystem::is_regular_file(atlas) && std::filesystem::file_size(atlas) > 0;
      run_status_ = published_poses_ > 0 && final_exported && latest_snapshot_ && !latest_snapshot_->points.empty() && atlas_saved ? "saved" : "incomplete";
    }
    writeTrackingReport();
  } catch (...) {
    run_status_ = "failed";
    writeTrackingReport();
    throw;
  }
  if(require_prepared_run_ && run_status_ != "saved")
    throw std::runtime_error("mapping incomplete: valid final map and nonempty Atlas required");
}

void StereoInertialNode::writeTrackingReport() const
{
  const std::filesystem::path final_path = trajectory_dir_ / "tracking.json";
  const std::filesystem::path temporary_path = trajectory_dir_ / "tracking.json.tmp";
  std::ofstream stream(temporary_path);
  if (!stream) {
    throw std::runtime_error("failed to open tracking report temporary file");
  }
  stream << "{\n"
         << "  \"status\": \"" << run_status_ << "\",\n"
         << "  \"pose_output\": \"" << (publish_base_ ? "base" : "camera") << "\",\n"
         << "  \"atlas_reuse_verified\": false,\n"
         << "  \"atlas_load_verified\": " << (atlas_load_verified_ ? "true" : "false") << ",\n"
         << "  \"invalid_map_drops\": " << invalid_map_drops_ << ",\n"
         << "  \"snapshot_skips\": " << snapshot_skips_ << ",\n"
         << "  \"image_pairs\": " << image_pairs_ << ",\n"
         << "  \"accepted_imu_samples\": " << accepted_imu_samples_ << ",\n"
         << "  \"rejected_imu_samples\": " << rejected_imu_samples_ << ",\n"
         << "  \"empty_imu_interval_drops\": " << empty_imu_interval_drops_ << ",\n"
         << "  \"conversion_failures\": " << conversion_failures_ << ",\n"
         << "  \"tracking_failures\": " << tracking_failures_ << ",\n"
         << "  \"published_poses\": " << published_poses_ << ",\n"
         << "  \"tracking_state_counts\": {";
  bool first = true;
  for (const auto & [state, count] : tracking_state_counts_) {
    stream << (first ? "\n" : ",\n")
           << "    \"" << state << "\": " << count;
    first = false;
  }
  if (!first) {
    stream << "\n  ";
  }
  stream << "}\n}\n";
  stream.close();
  if (!stream) {
    throw std::runtime_error("failed to write tracking report");
  }
  std::filesystem::rename(temporary_path, final_path);
}

void StereoInertialNode::publishDiagnostics() {
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "orbslam3/mapping"; status.hardware_id = "stereo";
  status.level = current_map_ && state_ == ORB_SLAM3::Tracking::OK && now().seconds()-last_pose_stamp_ < 1. ? 0 : 1;
  status.message = status.level == 0 ? "tracking" : "no valid current pose";
  const auto add = [&status](const std::string & key, const auto & value) {
    diagnostic_msgs::msg::KeyValue kv; kv.key = key; kv.value = std::to_string(value); status.values.push_back(kv);
  };
  add("tracking_state",state_); add("current_map_id",current_map_ ? static_cast<long long>(*current_map_) : -1LL);
  add("orb_runtime_map_id",orb_runtime_map_ ? static_cast<long long>(*orb_runtime_map_) : -1LL);
  add("last_valid_map_id",last_valid_map_ ? static_cast<long long>(*last_valid_map_) : -1LL);
  add("last_pose_source_stamp",last_pose_stamp_); add("last_pose_age_s",published_poses_ ? now().seconds()-last_pose_stamp_ : -1.);
  add("image_pairs",image_pairs_); add("conversion_drops",conversion_failures_);
  add("imu_interval_drops",empty_imu_interval_drops_); add("invalid_map_drops",invalid_map_drops_);
  add("tracking_failures",tracking_failures_); add("published_poses",published_poses_);
  add("last_image_age_s",last_image_stamp_ ? now().seconds()-*last_image_stamp_ : -1.);
  add("snapshot_skips",snapshot_skips_); add("process_ms",process_ms_); add("snapshot_ms",snapshot_ms_);
  add("snapshot_source_stamp", latest_snapshot_ ? latest_snapshot_->source_stamp : -1.);
  add("snapshot_map_id", latest_snapshot_ ? static_cast<long long>(latest_snapshot_->map_id) : -1LL);
  add("points",latest_snapshot_ ? latest_snapshot_->points.size() : 0U);
  add("keyframes",latest_snapshot_ ? latest_snapshot_->keyframes.size() : 0U);
  diagnostic_msgs::msg::KeyValue mode; mode.key="sensor_mode"; mode.value=requiresImu(sensor_mode_) ? "stereo_inertial" : "stereo";
  status.values.push_back(mode);
  diagnostic_msgs::msg::KeyValue output; output.key="pose_output"; output.value=publish_base_ ? "base" : "camera";
  status.values.push_back(output); array.status.push_back(status); diagnostics_publisher_->publish(array);
}
void StereoInertialNode::loadAlignments() {
  const auto file = trajectory_dir_ / "map_alignments.json";
  if (!std::filesystem::is_regular_file(file)) return;
  const cv::FileStorage storage(file.string(), cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (!storage.isOpened()) throw std::runtime_error("invalid map alignment metadata");
  for (auto it=storage.root().begin(); it!=storage.root().end(); ++it) {
    const auto node=*it; const auto rows=node["T_map_orb"];
    if (rows.size()!=4) throw std::runtime_error("invalid saved map transform");
    Eigen::Matrix4f matrix;
    for(int r=0;r<4;++r) {
      if(rows[r].size()!=4) throw std::runtime_error("invalid saved map transform row");
      for(int c=0;c<4;++c) matrix(r,c)=static_cast<float>(rows[r][c]);
    }
    if (!matrix.allFinite() || !(matrix.block<3,3>(0,0).transpose()*matrix.block<3,3>(0,0)).isApprox(Eigen::Matrix3f::Identity(),1e-4f) ||
        std::abs(matrix.block<3,3>(0,0).determinant()-1.f)>1e-4f || !matrix.row(3).isApprox(Eigen::RowVector4f(0,0,0,1),1e-5f))
      throw std::runtime_error("saved alignment is not SE(3)");
    alignments_.emplace(std::stoull(node.name()),Sophus::SE3f(matrix));
  }
}
void StereoInertialNode::saveAlignments() const {
  const auto file=trajectory_dir_/"map_alignments.json";
  std::ofstream out(file.string()+".tmp");
  out << "{\n" << std::setprecision(9);
  bool first=true;
  for(const auto & item : alignments_) {
    if(!first) out << ",\n";
    first=false;
    out << "\"" << item.first << "\":{\"provenance\":\"initial_body_pose_with_recorded_extrinsic\",\"T_map_orb\":[";
    const auto matrix=item.second.matrix();
    for(int r=0;r<4;++r) {
      if(r) out << ','; out << '[';
      for(int c=0;c<4;++c) { if(c) out << ','; out << matrix(r,c); }
      out << ']';
    }
    out << "]}";
  }
  out << "\n}\n"; out.close();
  if(!out) throw std::runtime_error("alignment write failed");
  std::filesystem::rename(file.string()+".tmp",file);
}

}  // namespace orbslam3_ros2
