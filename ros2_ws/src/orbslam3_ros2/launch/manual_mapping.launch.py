# ================================================================
# manual_mapping.launch.py
# 설명: 준비한 run의 cwd에서 visual stereo를 실행한다. 실기 시각이 기본이다.
# 사용법: ros2 launch orbslam3_ros2 manual_mapping.launch.py run_dir:=RUN vocabulary_path:=VOC
# ================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import hashlib
import json
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    arguments = [DeclareLaunchArgument('run_dir'), DeclareLaunchArgument('vocabulary_path'),
                 DeclareLaunchArgument('left_topic', default_value='/camera/camera/infra1/image_rect_raw'),
                 DeclareLaunchArgument('right_topic', default_value='/camera/camera/infra2/image_rect_raw'),
                 DeclareLaunchArgument('use_sim_time', default_value='false'),
                 DeclareLaunchArgument('image_preprocessing', default_value='raw')]
    return LaunchDescription([*arguments, OpaqueFunction(function=start_prepared_run)])


def start_prepared_run(context: object) -> list:
    run=Path(LaunchConfiguration('run_dir').perform(context))
    if not run.is_absolute(): raise ValueError('run_dir must be absolute')
    manifest=json.loads((run/'manifest.json').read_text())
    vocabulary=Path(LaunchConfiguration('vocabulary_path').perform(context))
    digest=hashlib.sha256()
    with vocabulary.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''): digest.update(block)
    if digest.hexdigest()!=manifest['calibration']['vocabulary_sha256']:
        raise ValueError('vocabulary differs from prepared calibration')
    if (run/'mapping.started').exists() or (run/'atlas.osa').exists():
        raise ValueError('run already used; prepare a new run and load prior Atlas explicitly')
    node = Node(package='orbslam3_ros2', executable='orbslam3_stereo_inertial', output='screen',
                cwd=LaunchConfiguration('run_dir'), parameters=[{
                    'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                    'require_prepared_run': True, 'sensor_mode': 'stereo', 'image_preprocessing': LaunchConfiguration('image_preprocessing'),
                    'settings_path': PathJoinSubstitution([LaunchConfiguration('run_dir'), 'orb.yaml']),
                    'vocabulary_path': LaunchConfiguration('vocabulary_path'),
                    'trajectory_dir': LaunchConfiguration('run_dir'),
                    'left_topic': LaunchConfiguration('left_topic'), 'right_topic': LaunchConfiguration('right_topic')}])
    return [node]
