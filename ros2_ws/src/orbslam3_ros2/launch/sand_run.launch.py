# ================================================================
# sand_run.launch.py
# 설명: 모래 지형 stereo/IMU topic으로 ORB-SLAM3 adapter를 실행한다.
# 사용법:
#   ros2 launch orbslam3_ros2 sand_run.launch.py vocabulary_path:=... settings_path:=... trajectory_dir:=...
# ================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """필수 파일 경로와 고정 sensor topic을 받는 launch description을 만든다."""
    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("vocabulary_path"),
        DeclareLaunchArgument("settings_path"),
        DeclareLaunchArgument("trajectory_dir"),
        DeclareLaunchArgument("left_topic", default_value="/rover/stereo/left/image_raw"),
        DeclareLaunchArgument("right_topic", default_value="/rover/stereo/right/image_raw"),
        DeclareLaunchArgument("imu_topic", default_value="/rover/imu/data"),
    ]
    node = Node(
        package="orbslam3_ros2",
        executable="orbslam3_stereo_inertial",
        name="orbslam3_stereo_inertial",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "vocabulary_path": LaunchConfiguration("vocabulary_path"),
                "settings_path": LaunchConfiguration("settings_path"),
                "trajectory_dir": LaunchConfiguration("trajectory_dir"),
                "left_topic": LaunchConfiguration("left_topic"),
                "right_topic": LaunchConfiguration("right_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
            }
        ],
    )
    return LaunchDescription([*arguments, node])
