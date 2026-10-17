# ================================================================
# vio_offline.py
# 설명: 저장한 시뮬 stereo+IMU 기록으로 ORB-SLAM3 stereo VO를 돌리고, 열화한 IMU와 ESKF로 융합해 GT 대비 오차를 비교한다.
# 사용법:
#   python -m experiments.site_sim.vio_offline orb STEREO_DIR OUT_DIR
#   python -m experiments.site_sim.vio_offline fuse STEREO_DIR ORB_DIR OUT_DIR --imu avg50 [--gaps 40 41.5]
# ================================================================
from pathlib import Path
import argparse
import csv
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
BASELINE = Path('/home/su/rover/outputs/handheld_mapping/20261015_tracking_recovery/baseline')
PANGOLIN = Path('/home/su/rover/outputs/handheld_mapping/native_dependencies/pangolin/lib')
VOCABULARY = ROOT/'external/ORB-LINE-SLAM/Vocabulary/ORBvoc.txt'
URDF = ROOT/'outputs/site_sim/rover_assets_new/rover_portable.urdf'
GEOMETRY = ROOT/'outputs/site_sim/rover_assets_new/config/swerve_geometry.json'


def read_gt(stereo: Path) -> dict[int, np.ndarray]:
    """gt.jsonl의 base pose(4x4)를 시각(ns)별로 읽는다."""
    poses = {}
    for line in (stereo/'gt.jsonl').read_text().splitlines():
        row = json.loads(line)
        q = row['quaternion_wxyz']
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
        pose[:3, 3] = row['position_m']
        poses[row['stamp_ns']] = pose
    return poses


def orb_calibration(stereo: Path, imu_hz: float) -> dict:
    """시뮬 보정에서 ORB 설정 키(내부 파라미터, 좌우 외부 파라미터, IMU 외부 파라미터)를 만든다."""
    from experiments.orbslam3_offline.calibration import CameraIntrinsics, build_orb_calibration
    sim = json.loads((stereo/'calibration.json').read_text())
    calibration = build_orb_calibration(URDF, CameraIntrinsics(640, 480, sim['fx'], sim['fy'], sim['cx'], sim['cy']),
                                        camera_hz=sim['requested_hz'], imu_hz=imu_hz)
    calibration['Stereo.T_c1_c2'] = sim['T_left_right']
    calibration['ROS.T_c1_base'] = np.linalg.inv(np.asarray(sim['T_base_left_optical'])).tolist()
    calibration['ROS.pose_output'] = 'camera'
    return calibration


def wheel_odometry(telemetry: Path, geometry: Path, out: Path) -> None:
    """telemetry의 조향각·바퀴 각속도에서 body twist를 구해 t_s,vx,vy,wz CSV로 저장한다.

    Isaac DOF 배열은 [rocker 2, 조향 4, 바퀴 4]이고 조향·바퀴 모두 [FL, RL, FR, RR] 순서다
    (명령값과의 상관으로 확인). SwerveGeometry.modules 순서는 [FL, FR, RL, RR]이라 슬롯을 바꿔 읽는다.
    """
    from experiments.orbslam3_ros2.swerve import SwerveGeometry, body_twist_from_modules
    modules = SwerveGeometry.from_json(geometry)
    slots = [0, 2, 1, 3]
    with out.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['t_s', 'vx', 'vy', 'wz'])
        last = -1.
        for line in telemetry.read_text().splitlines():
            row = json.loads(line)
            if row['stamp_s'] <= last:
                continue
            last = row['stamp_s']
            q, w = row['joint_positions'], row['joint_velocities']
            vx, vy, wz, _ = body_twist_from_modules(modules, [q[2+slot] for slot in slots], [w[6+slot] for slot in slots])
            writer.writerow([f'{last:.6f}', f'{vx:.6f}', f'{vy:.6f}', f'{wz:.6f}'])


def write_yaml(path: Path, calibration: dict) -> None:
    from experiments.orbslam3_offline.calibration import write_orb_yaml
    write_orb_yaml(path, calibration)


def run_orb(stereo: Path, out: Path, timeout_s: float) -> None:
    """GT 없이 좌우 영상·보정·시각만으로 ORB-SLAM3 stereo를 실제 시각 간격으로 돌린다."""
    out.mkdir(parents=True, exist_ok=False)
    write_yaml(out/'orb.yaml', orb_calibration(stereo, 200.))
    rows = list(csv.reader((stereo/'mav0/cam0/data.csv').open()))[1:]
    with (out/'pairs.tsv').open('w') as stream:
        for stamp, name in rows:
            stream.write(f'{stamp}\t{stereo}/mav0/cam0/data/{name}\t{stereo}/mav0/cam1/data/{name}\n')
    env = {**os.environ, 'LD_LIBRARY_PATH': f'{PANGOLIN}:{BASELINE/"runtime"}', 'OMP_NUM_THREADS': '4', 'OPENBLAS_NUM_THREADS': '1'}
    lines = (out/'pairs.tsv').read_text().splitlines()
    first = int(lines[0].split('\t')[0])
    with (out/'run.log').open('w') as log:
        process = subprocess.Popen([str(BASELINE/'rover_orb3_compare'), str(VOCABULARY), 'unused', str(out/'orb.yaml'), '-', str(out)],
                                   stdin=subprocess.PIPE, stdout=log, stderr=log, text=True, env=env)
        try:
            deadline = time.monotonic()+60
            while not (out/'timings.csv').exists():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError('ORB runner did not become ready')
                time.sleep(.1)
            start = time.monotonic()
            for line in lines:  # 원래 시뮬레이션 시각 간격(1x)으로 투입해 local mapping 시간을 현실과 같게 한다.
                wait = start+(int(line.split('\t')[0])-first)*1e-9-time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                process.stdin.write(line+'\n')
                process.stdin.flush()
            process.stdin.close()
            process.wait(timeout=timeout_s)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)


def orb_for_fusion(stereo: Path, orb: Path, out: Path) -> None:
    """ORB live_poses.csv를 필터가 읽는 tracking.csv와 base_trajectory.tum(base pose)으로 바꾼다."""
    calibration = json.loads((stereo/'calibration.json').read_text())
    cam_from_base = np.linalg.inv(np.asarray(calibration['T_base_left_optical']))
    out.mkdir(parents=True, exist_ok=False)
    tracking, trajectory = [], []
    for row in csv.DictReader((orb/'live_poses.csv').open()):
        tracking.append({'timestamp_ns': row['stamp_ns'], 'state': row['state'], 'map_id': row['map_id'], 'imu_samples': 0})
        if row['state'] == '2' and int(row['map_id']) >= 0:
            world_from_cam = np.array([float(row[f'm{i}{j}']) for i in range(4) for j in range(4)]).reshape(4, 4)
            base = world_from_cam @ cam_from_base
            q = Rotation.from_matrix(base[:3, :3]).as_quat()
            trajectory.append(f'{int(row["stamp_ns"])*1e-9:.9f} {" ".join(f"{v:.9f}" for v in [*base[:3, 3], *q])}')
    with (out/'tracking.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['timestamp_ns', 'state', 'map_id', 'imu_samples'])
        writer.writeheader()
        writer.writerows(tracking)
    (out/'base_trajectory.tum').write_text('\n'.join(trajectory)+'\n')
    (out/'complete.txt').write_text('complete\n')


def ideal_imu_from_telemetry(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """20Hz 제어 telemetry의 GT pose를 미분해 200Hz 이상적 IMU를 만든다."""
    from experiments.site_sim.imu_from_poses import ideal_imu_from_poses
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    times = np.array([r['stamp_s'] for r in rows])
    keep = np.r_[True, np.diff(times) > 1e-6]
    times = times[keep]
    positions = np.array([r['position_m'] for r in rows])[keep]
    quaternions = np.array([r['quaternion_wxyz'] for r in rows])[keep]
    return ideal_imu_from_poses(times, positions, quaternions, np.arange(times[0], times[-1], .005))


def fusion_sequence(stereo: Path, out: Path, *, average: bool, noise: bool, delay_s: float, gaps_s: tuple, seed: int) -> None:
    """영상은 링크하고 IMU만 열화한 필터 입력 sequence를 만든다."""
    from experiments.site_sim.imu_degrade import degrade
    out.mkdir(parents=True, exist_ok=False)
    for side in ('0', '1'):
        (out/f'mav0/cam{side}').parent.mkdir(parents=True, exist_ok=True)
        (out/f'mav0/cam{side}').symlink_to((stereo/f'mav0/cam{side}/data').parent.resolve(), target_is_directory=True)
    (out/'mav0/imu0').mkdir(parents=True, exist_ok=True)
    stamps = [int(r[0]) for r in list(csv.reader((stereo/'mav0/cam0/data.csv').open()))[1:]]
    (out/'camera_times.txt').write_text(''.join(f'{s}\n' for s in stamps))
    ideal_times, ideal = ideal_imu_from_telemetry(stereo.parent/'telemetry.jsonl')
    times, values = degrade(ideal_times, ideal, average=average, noise=noise, delay_s=delay_s, gaps_s=gaps_s, seed=seed)
    pad = .2
    times = np.r_[times[0]-pad, times, times[-1]+pad]
    values = np.vstack([values[0], values, values[-1]])
    with (out/'mav0/imu0/data.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['#timestamp [ns]', 'wx', 'wy', 'wz', 'ax', 'ay', 'az'])
        writer.writerows([int(round(t*1e9)), *v] for t, v in zip(times, values))
    calibration = orb_calibration(stereo, 50. if average else 200.)
    (out/'calibration.json').write_text(json.dumps(calibration, indent=2)+'\n')


def pose_errors(stereo: Path, tum: Path) -> dict:
    """TUM base pose를 GT와 직접 비교한다(초기 pose는 필터가 알려진 시작값을 쓴다)."""
    gt = read_gt(stereo)
    stamps = np.array(sorted(gt))
    rows = np.loadtxt(tum)
    errors = []
    for row in rows:
        ns = stamps[int(np.argmin(abs(stamps*1e-9-row[0])))]
        errors.append(np.linalg.norm(row[1:4]-gt[int(ns)][:3, 3]))
    errors = np.array(errors)*100
    return {'rmse_cm': float(np.sqrt((errors**2).mean())), 'last_cm': float(errors[-1]), 'max_cm': float(errors.max())}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('orb', 'fuse'))
    parser.add_argument('stereo', type=Path)
    parser.add_argument('first', type=Path)
    parser.add_argument('second', type=Path, nargs='?')
    parser.add_argument('--imu', choices=('ideal', 'avg50'), default='avg50')
    parser.add_argument('--gaps', type=float, nargs=2, action='append', default=[])
    parser.add_argument('--delay-s', type=float, default=0.)
    parser.add_argument('--encoder', action='store_true', help='바퀴 엔코더 융합 필터를 함께 계산')
    parser.add_argument('--timeout-s', type=float, default=400.)
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()
    if args.action == 'orb':
        run_orb(args.stereo, args.first, args.timeout_s)
        return
    if args.second is None:
        parser.error('fuse needs STEREO ORB OUT')
    out = args.second
    out.mkdir(parents=True, exist_ok=False)
    average = args.imu == 'avg50'
    fusion_sequence(args.stereo, out/'sequence', average=average, noise=average, delay_s=args.delay_s,
                    gaps_s=tuple(tuple(g) for g in args.gaps), seed=args.seed)
    orb_for_fusion(args.stereo, args.first, out/'orb')
    first = read_gt(args.stereo)
    stamp = sorted(first)[0]
    (out/'prior.json').write_text(json.dumps({'world_from_base': first[stamp].tolist()}))
    wheel_odometry(args.stereo.parent/'telemetry.jsonl', GEOMETRY, out/'wheel_odometry.csv')
    from experiments.localization.fusion import run
    run(out/'sequence', out/'orb', out/'prior.json', out/'estimate', reanchor_maps=True,
        encoder=out/'wheel_odometry.csv' if args.encoder else None)
    names = ('imu_only', 'visual_imu') + (('visual_imu_enc',) if args.encoder else ())
    result = {name: pose_errors(args.stereo, out/'estimate'/f'{name}.tum') for name in names}
    (out/'errors.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
