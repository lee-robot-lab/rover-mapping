#!/usr/bin/env python3
# ================================================================
# mapping_preflight.py
# 설명: 호스트 설치 상태와 정지 stereo probe를 읽기 전용으로 검사한다. 센서·모터를 실행하지 않는다.
# 사용법: mapping_preflight.py host
#   mapping_preflight.py sensors --probe probe.json --urdf rover.urdf --profile 640x480x30:Y8 --rectified-input
# ================================================================
import argparse
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
from prepare_mapping_run import atomic_json, calibration_from_probe, validate_stereo_info


def host_snapshot() -> dict:
    packages={}
    for package in ('realsense2_camera','orbslam3_ros2','rosbag2_storage_mcap','foxglove_bridge'):
        try:
            result=subprocess.run(['ros2','pkg','prefix',package],capture_output=True,text=True,timeout=5)
            packages[package]={'available':result.returncode==0,'prefix':result.stdout.strip() if result.returncode==0 else None}
        except (OSError,subprocess.TimeoutExpired):
            packages[package]={'available':False,'prefix':None}
    os_release=platform.freedesktop_os_release()
    target=(os_release.get('ID')=='ubuntu' and os_release.get('VERSION_ID')=='24.04'
            and platform.machine()=='aarch64' and os.environ.get('ROS_DISTRO')=='jazzy')
    return {'scope':'read_only_host_inventory','os':os_release,'architecture':platform.machine(),
            'ros_distro':os.environ.get('ROS_DISTRO'),'python':platform.python_version(),'packages':packages,
            'disk_free_bytes':shutil.disk_usage(Path.cwd()).free,
            'software_ready_for_pi':target and all(item['available'] for item in packages.values()),
            'sensor_verified':False,'spi_owner_verified':False,'motor_test_performed':False}


def assess_probe(probe: dict, urdf: Path, profile: str, rectified: bool,
                 max_age_s: float=.5, min_free_bytes: int=2_000_000_000) -> dict:
    issues=[]; warnings=['URDF extrinsic is provisional; camera pose and control require physical checks']
    calibration=None
    try:
        calibration=calibration_from_probe(probe,urdf,profile,rectified)
        validate_stereo_info(calibration['left'],calibration['right'])
        if probe['status']!='captured' or not math.isfinite(probe['elapsed_s']) or probe['elapsed_s']<29.95:
            issues.append('at least 30 seconds of captured input required')
        if probe['clock']['use_sim_time'] is not False:
            issues.append('real preflight requires use_sim_time=false')
        counts=[]
        for side in ('left','right'):
            stats=probe['stats'][side]; counts.append(stats['count'])
            if (calibration[side]['width'],calibration[side]['height'])!=(stats['width'],stats['height']):
                issues.append(f'{side}: CameraInfo/image dimensions disagree')
            if stats['count']<=0 or stats['non_increasing'] or stats['profile_changes']:
                issues.append(f'{side}: missing/backward stamps or changing image profile')
            ages=(stats['min_age_s'],stats['max_age_s'])
            if not all(isinstance(age,(float,int)) and math.isfinite(age) for age in ages) or ages[0]<-.05 or ages[1]>max_age_s:
                issues.append(f'{side}: unknown/negative/excessive image age')
        window=min(1000,max(counts))
        if window<=0 or probe['exact_pair_candidates']<.99*window:
            issues.append('less than 99 percent exact source pairs in last probe window')
        system=probe['system']
        if not system or min(item['disk_free_bytes'] for item in system)<min_free_bytes:
            issues.append('insufficient or unknown storage headroom')
        temperatures=[float(value)/1000 for item in system for value in item['temperatures_mC'].values()]
        if not temperatures: warnings.append('temperature unavailable')
        elif max(temperatures)>=80: warnings.append('temperature reached 80 C; inspect cooling and load')
    except (KeyError,TypeError,ValueError,OSError) as error:
        issues.append(f'incomplete or invalid evidence: {error}')
    return {'scope':'stationary_sensor_capture_only','capture_ready':not issues,'issues':issues,'warnings':warnings,
            'extrinsic_verified':False,'pitch_down_deg':calibration['pitch_down_deg'] if calibration else None,
            'imu_clock_verified':False,'thresholds':{'max_age_s':max_age_s,'min_free_bytes':min_free_bytes,
                'min_duration_s':30.,'min_exact_pair_fraction':.99},'motor_test_performed':False}


def main() -> None:
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path)
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('host')
    sensors=sub.add_parser('sensors')
    for name in ('probe','urdf'): sensors.add_argument('--'+name,type=Path,required=True)
    sensors.add_argument('--profile',required=True)
    sensors.add_argument('--rectified-input',action='store_true')
    sensors.add_argument('--max-age-s',type=float,default=.5)
    sensors.add_argument('--min-free-bytes',type=int,default=2_000_000_000)
    args=parser.parse_args()
    if args.output and args.output.exists(): parser.error('output exists; use a new path')
    if args.command=='host':
        result=host_snapshot(); ready=result['software_ready_for_pi']
    else:
        if not math.isfinite(args.max_age_s) or args.max_age_s<=0 or args.min_free_bytes<=0:
            parser.error('positive finite age and storage thresholds required')
        result=assess_probe(json.loads(args.probe.read_text()),args.urdf,args.profile,args.rectified_input,
                            args.max_age_s,args.min_free_bytes)
        ready=result['capture_ready']
    if args.output: atomic_json(args.output,result)
    print(json.dumps(result,indent=2,allow_nan=False))
    raise SystemExit(0 if ready else 1)


if __name__=='__main__': main()
