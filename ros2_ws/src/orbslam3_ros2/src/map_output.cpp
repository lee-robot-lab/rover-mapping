// ================================================================
// map_output.cpp
// 설명: map별 frame을 유지하며 유효한 지도 값만 ROS와 PLY/CSV로 내보낸다.
// 사용법: exportMap(run_dir, snapshot, T_map_orb)
// ================================================================
#include "orbslam3_ros2/map_output.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <rclcpp/time.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
namespace orbslam3_ros2 {
namespace {
std_msgs::msg::Header header(const MapSnapshot & s) {
  std_msgs::msg::Header h;
  h.frame_id = "orb_map_" + std::to_string(s.map_id);
  h.stamp = rclcpp::Time(static_cast<int64_t>(s.source_stamp * 1e9));
  return h;
}
}
sensor_msgs::msg::PointCloud2 makeMapCloud(const MapSnapshot & s, const Sophus::SE3f & t) {
  sensor_msgs::msg::PointCloud2 out;
  out.header = header(s);
  sensor_msgs::PointCloud2Modifier modifier(out);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  const auto count = std::count_if(s.points.begin(), s.points.end(), [](const auto & p) {return p.position.allFinite();});
  modifier.resize(count);
  sensor_msgs::PointCloud2Iterator<float> x(out, "x"), y(out, "y"), z(out, "z");
  for (const auto & p : s.points) {
    if (!p.position.allFinite()) continue;
    const auto v = t * p.position;
    *x = v.x(); *y = v.y(); *z = v.z(); ++x; ++y; ++z;
  }
  out.is_dense = true;
  return out;
}
geometry_msgs::msg::PoseArray makeKeyframePoses(const MapSnapshot & s, const Sophus::SE3f & t) {
  geometry_msgs::msg::PoseArray out;
  out.header = header(s);
  for (const auto & frame : s.keyframes) {
    if (!frame.T_wc1.matrix().allFinite()) continue;
    const auto pose = t * frame.T_wc1;
    geometry_msgs::msg::Pose p;
    p.position.x = pose.translation().x(); p.position.y = pose.translation().y(); p.position.z = pose.translation().z();
    const auto q = pose.unit_quaternion();
    p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z(); p.orientation.w = q.w();
    out.poses.push_back(p);
  }
  return out;
}
void exportMap(const std::filesystem::path & dir, const MapSnapshot & s, const Sophus::SE3f & t) {
  const auto prefix = "map_" + std::to_string(s.map_id);
  const auto ply = dir / (prefix + ".ply"), csv = dir / (prefix + "_keyframes.csv");
  std::ofstream points(ply.string() + ".tmp"), frames(csv.string() + ".tmp");
  if (!points || !frames) throw std::runtime_error("cannot open map export");
  const auto count = std::count_if(s.points.begin(), s.points.end(), [](const auto & p) {return p.position.allFinite();});
  points << "ply\nformat ascii 1.0\ncomment frame orb_map_" << s.map_id
    << "\nelement vertex " << count << "\nproperty float x\nproperty float y\nproperty float z\nend_header\n" << std::setprecision(9);
  for (const auto & p : s.points) if (p.position.allFinite()) {
    auto v = t * p.position; points << v.x() << ' ' << v.y() << ' ' << v.z() << '\n';
  }
  frames << "map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n" << std::setprecision(17);
  auto sorted = s.keyframes;
  std::sort(sorted.begin(), sorted.end(), [](const auto & a, const auto & b) {return a.stamp < b.stamp;});
  for (const auto & f : sorted) {
    auto pose = t * f.T_wc1; auto p = pose.translation(); auto q = pose.unit_quaternion();
    frames << s.map_id << ',' << f.id << ',' << f.stamp << ',' << p.x() << ',' << p.y() << ',' << p.z()
      << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << '\n';
  }
  points.close(); frames.close();
  if (!points || !frames) throw std::runtime_error("map export write failed");
  std::filesystem::rename(ply.string() + ".tmp", ply);
  std::filesystem::rename(csv.string() + ".tmp", csv);
}
}
