#!/usr/bin/env python3
# ================================================================
# prepare_mapping_run.py
# 설명: 실제 CameraInfo와 명시된 장착 보정으로 새 run과 ORB stereo 설정을 만든다.
# 사용법: prepare_mapping_run.py --calibration sensor.json --urdf robot.urdf --vocabulary ORBvoc.txt --run-dir RUN
# ================================================================
import argparse
import copy
import hashlib
import json
import math
import re
import shutil
from pathlib import Path
import numpy as np


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def validate_stereo_info(left: dict, right: dict) -> None:
    for info in (left, right):
        if any(not isinstance(info.get(key), int) or info[key] <= 0 for key in ('width', 'height')):
            raise ValueError('CameraInfo width/height must be positive integers')
        k = np.asarray(info.get('k', []), dtype=float)
        d = np.asarray(info.get('d', []), dtype=float)
        if k.shape != (9,) or not np.isfinite(k).all() or k[0] <= 0 or k[4] <= 0:
            raise ValueError('invalid CameraInfo K/focal length')
        if not np.allclose(k[[1, 3, 6, 7, 8]], [0, 0, 0, 0, 1]):
            raise ValueError('unsupported skew/homogeneous K')
        if not 0 <= k[2] < info['width'] or not 0 <= k[5] < info['height']:
            raise ValueError('principal point outside image')
        if info.get('distortion_model') != 'plumb_bob' or d.shape != (5,) or not np.isfinite(d).all():
            raise ValueError('only finite five-coefficient plumb_bob CameraInfo supported')
        if info.get('binning_x', 0) not in (0, 1) or info.get('binning_y', 0) not in (0, 1):
            raise ValueError('binned CameraInfo requires explicit new calibration')
        if any(info.get('roi', {}).get(k, 0) for k in ('x_offset', 'y_offset', 'width', 'height')):
            raise ValueError('cropped CameraInfo requires explicit new calibration')
    if (left['width'], left['height']) != (right['width'], right['height']):
        raise ValueError('stereo dimensions differ')


def validate_transform(value: list, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f'{name}: finite 4x4 required')
    r = matrix[:3, :3]
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6) or not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or not math.isclose(np.linalg.det(r), 1., abs_tol=1e-5):
        raise ValueError(f'{name}: invalid SE(3)')
    return matrix


def camera_contract(calibration: dict) -> dict:
    result = {key: calibration[key] for key in ('T_c1_c2', 'camera_profile', 'vocabulary_sha256')}
    result['pose_output'] = calibration.get('pose_output', 'base')
    for side in ('left', 'right'):
        result[side] = {key: calibration[side][key] for key in ('width', 'height', 'k', 'd', 'distortion_model')}
    return result


def orb_yaml(calibration: dict, load: bool) -> str:
    match = re.fullmatch(r'(\d+)x(\d+)x(\d+):Y8', calibration['camera_profile'])
    values = {'File.version': '1.0', 'Camera.type': 'PinHole',
              'ROS.pose_output': calibration.get('pose_output', 'base'),
              'Camera.width': int(match[1]), 'Camera.height': int(match[2]),
              'Camera.fps': int(match[3]), 'Camera.RGB': 0, 'Stereo.ThDepth': 40.,
              'ORBextractor.nFeatures': 1200, 'ORBextractor.scaleFactor': 1.2,
              'ORBextractor.nLevels': 8, 'ORBextractor.iniThFAST': 20,
              'ORBextractor.minThFAST': 7, 'System.SaveAtlasToFile': 'atlas',
              'Viewer.KeyFrameSize': .05, 'Viewer.KeyFrameLineWidth': 1., 'Viewer.GraphLineWidth': .9,
              'Viewer.PointSize': 2., 'Viewer.CameraSize': .08, 'Viewer.CameraLineWidth': 3.,
              'Viewer.ViewpointX': 0., 'Viewer.ViewpointY': -.7, 'Viewer.ViewpointZ': -1.8,
              'Viewer.ViewpointF': 500., 'Viewer.imageViewScale': 1.}
    if load:
        values['System.LoadAtlasFromFile'] = 'atlas_input'
    for index, side in enumerate(('left', 'right'), 1):
        info = calibration[side]
        for name, value in zip(('fx', 'fy', 'cx', 'cy'), (info['k'][0], info['k'][4], info['k'][2], info['k'][5])):
            values[f'Camera{index}.{name}'] = float(value)
        for name, value in zip(('k1', 'k2', 'p1', 'p2', 'k3'), info['d']):
            values[f'Camera{index}.{name}'] = float(value)
    lines = ['%YAML:1.0', '---']
    for key, value in values.items():
        lines.append(f'{key}: {json.dumps(value)}')
    transforms = [('Stereo.T_c1_c2', calibration['T_c1_c2'])]
    if calibration.get('pose_output', 'base') == 'base':
        transforms.append(('ROS.T_c1_base', calibration['T_c1_base']))
    for key, value in transforms:
        numbers = ', '.join(format(float(v), '.12g') for v in np.asarray(value).flat)
        lines += [f'{key}: !!opencv-matrix', '  rows: 4', '  cols: 4', '  dt: f', f'  data: [ {numbers} ]']
    return '\n'.join(lines) + '\n'


def prepare_run(run_dir: Path, calibration: dict, urdf: Path | None, load_atlas: Path | None = None) -> dict:
    calibration = copy.deepcopy(calibration)
    validate_stereo_info(calibration['left'], calibration['right'])
    stereo = validate_transform(calibration['T_c1_c2'], 'T_c1_c2')
    pose_output = calibration.get('pose_output', 'base')
    if pose_output not in ('base', 'camera'):
        raise ValueError('pose_output must be base or camera')
    if pose_output == 'camera':
        if urdf is not None or any(k in calibration for k in ('T_c1_base', 'pitch_down_deg', 'extrinsic_source', 'extrinsic_verified')):
            raise ValueError('handheld camera output must not claim robot geometry')
    else:
        validate_transform(calibration['T_c1_base'], 'T_c1_base')
        if not math.isfinite(calibration['pitch_down_deg']) or calibration['extrinsic_source'] not in ('urdf_provisional', 'measured'):
            raise ValueError('explicit pitch and extrinsic provenance required')
        if not isinstance(calibration['extrinsic_verified'], bool):
            raise ValueError('explicit extrinsic verification flag required')
        if calibration['extrinsic_source'] == 'urdf_provisional' and calibration['extrinsic_verified']:
            raise ValueError('provisional extrinsic cannot be verified')
        if urdf is None:
            raise ValueError('base output requires URDF')
        urdf = Path(urdf).resolve(strict=True)
    if stereo[0,3] <= 0 or stereo[0,3] < max(abs(stereo[1,3]),abs(stereo[2,3])):
        raise ValueError('positive horizontal left-to-right baseline required; check camera order')
    profile = re.fullmatch(r'(\d+)x(\d+)x(\d+):Y8', calibration.get('camera_profile', ''))
    if not profile or int(profile[3]) <= 0 or (int(profile[1]), int(profile[2])) != (calibration['left']['width'], calibration['left']['height']):
        raise ValueError('actual camera profile and CameraInfo disagree')
    if not isinstance(calibration['imu_clock_verified'], bool):
        raise ValueError('explicit verification flags required')
    if not calibration.get('vocabulary_sha256'):
        raise ValueError('vocabulary checksum required')
    alignment = None
    if load_atlas is not None:
        load_atlas = Path(load_atlas).resolve(strict=True)
        if load_atlas.stat().st_size == 0:
            raise ValueError('empty Atlas')
        previous = json.loads((load_atlas.parent / 'manifest.json').read_text())
        if previous.get('map_alignment_identity') != 'origin_keyframe_id_v1':
            raise ValueError('legacy mutable-map-ID alignment requires explicit migration; start a new map')
        if camera_contract(previous['calibration']) != camera_contract(calibration):
            raise ValueError('Atlas camera/vocabulary contract mismatch; start a new map')
        alignment = load_atlas.parent / 'map_alignments.json'
        if not alignment.is_file():
            raise ValueError('Atlas alignment metadata missing')
        saved=json.loads(alignment.read_text())
        if not isinstance(saved,dict): raise ValueError('invalid saved map metadata')
        for map_id,value in saved.items():
            if not map_id.isdecimal() or not 0 <= int(map_id)<2**64:
                raise ValueError('invalid saved map id')
            validate_transform(value.get('T_map_orb',[]),'saved T_map_orb')
    run_dir = Path(run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=False)
    if urdf is not None:
        shutil.copyfile(urdf, run_dir / 'robot.urdf')
    (run_dir / 'orb.yaml').write_text(orb_yaml(calibration, load_atlas is not None))
    atomic_json(run_dir / 'calibration.json', calibration)
    if alignment is not None:
        shutil.copyfile(alignment, run_dir / 'map_alignments.json')
    else:
        atomic_json(run_dir / 'map_alignments.json', {})
    manifest = {'schema': 1, 'map_alignment_identity':'origin_keyframe_id_v1', 'status': 'prepared', 'sensor_mode': 'stereo', 'run_dir': str(run_dir),
                'calibration': calibration, 'urdf_sha256': sha256(urdf) if urdf else None, 'pose_output': pose_output,
                'atlas_load_requested': load_atlas is not None, 'atlas_reuse_verified': False,
                'orb_commit': '4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4', 'imu_usage': 'unavailable' if pose_output == 'camera' else 'recording_receipt_only'}
    if load_atlas is not None:
        shutil.copyfile(load_atlas, run_dir / 'atlas_input.osa')
        manifest['atlas_input_sha256'] = sha256(run_dir / 'atlas_input.osa')
    atomic_json(run_dir / 'manifest.json', manifest)
    return manifest


def calibration_from_probe(probe: dict, urdf: Path | None, profile: str, rectified_confirmed: bool) -> dict:
    """실제 rectified P를 사용하며 handheld는 로버 장착 변환을 만들지 않는다."""
    if not rectified_confirmed:
        raise ValueError('confirm rectified input explicitly; raw CameraInfo cannot be relabeled')
    info = probe['camera_info']
    match = re.fullmatch(r'(\d+)x(\d+)x(\d+):Y8', profile)
    if not match or probe.get('exact_pair_candidates',0) <= 0:
        raise ValueError('actual profile and exact stereo input required')
    projections = [np.asarray(info[side]['p'],dtype=float).reshape(3,4) for side in ('left','right')]
    for side,p in zip(('left','right'), projections):
        measured = probe['stats'][side]
        if not np.isfinite(p).all() or p[0,0] <= 0 or not np.allclose(p[2],[0,0,1,0]) or not np.allclose(p[[0,1,1],[1,0,3]],[0,0,0]):
            raise ValueError('unsupported rectified projection')
        if measured['encoding'] != 'mono8' or (measured['width'],measured['height']) != (int(match[1]),int(match[2])) or not .8*int(match[3]) <= measured['receipt_rate_hz'] <= 1.2*int(match[3]):
            raise ValueError('actual image profile/rate mismatch')
    if not np.allclose(projections[0][:,:3],projections[1][:,:3],atol=1e-4) or abs(projections[0][0,3])>1e-6:
        raise ValueError('rectified projections must share K and left origin')
    baseline = -projections[1][0,3]/projections[1][0,0]
    if not baseline > 0: raise ValueError('invalid left/right projection baseline sign')
    calibration={}
    for side,p in zip(('left','right'),projections):
        calibration[side]={'width':info[side]['width'],'height':info[side]['height'], 'k':p[:,:3].reshape(-1).tolist(),
                           'd':[0.]*5,'distortion_model':'plumb_bob'}
    stereo=np.eye(4); stereo[0,3]=baseline
    calibration.update(T_c1_c2=stereo.tolist(),imu_clock_verified=False,
        camera_profile=profile,input_geometry='rectified_confirmed',source_camera_info=info,
        stereo_extrinsic_source='rectified_CameraInfo_P')
    if urdf is None:
        calibration['pose_output']='camera'
        return calibration
    from xml.etree import ElementTree
    from collections import deque
    graph: dict[str,list] = {}
    for joint in ElementTree.parse(urdf).getroot().findall('joint'):
        parent,child=joint.find('parent').get('link'),joint.find('child').get('link')
        origin=joint.find('origin'); xyz=[float(v) for v in origin.get('xyz','0 0 0').split()] if origin is not None else [0.]*3
        roll,pitch,yaw=[float(v) for v in origin.get('rpy','0 0 0').split()] if origin is not None else [0.]*3
        cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
        t=np.eye(4); t[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]; t[:3,3]=xyz
        graph.setdefault(parent,[]).append((child,t,joint.get('type')))
        graph.setdefault(child,[]).append((parent,np.linalg.inv(t),joint.get('type')))
    left_frames = {'camera_infra1_optical_frame', 'camera_left_optical_frame'}
    queue=deque([('base_link',np.eye(4))]); seen=set(); candidates={}
    while queue:
        frame,t=queue.popleft()
        if frame in seen: continue
        seen.add(frame)
        if frame in left_frames: candidates[frame]=t
        for child,edge,kind in graph.get(frame,[]):
            if kind=='fixed': queue.append((child,t@edge))
    if not candidates: raise ValueError('URDF fixed path to left optical frame missing')
    if len(candidates) != 1: raise ValueError('URDF left optical frame ambiguous')
    left_frame, base_c1 = next(iter(candidates.items()))
    rectification=np.eye(4); rectification[:3,:3]=np.asarray(info['left']['r']).reshape(3,3)
    validate_transform(rectification.tolist(),'CameraInfo.R')
    forward=base_c1[:3,2]
    calibration.update(T_c1_base=(rectification@np.linalg.inv(base_c1)).tolist(),
        pitch_down_deg=math.degrees(math.atan2(-forward[2],math.hypot(*forward[:2]))),
        extrinsic_source='urdf_provisional',extrinsic_verified=False,
        urdf_left_optical_frame=left_frame)
    return calibration


def main() -> None:
    parser = argparse.ArgumentParser()
    inputs=parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--calibration',type=Path)
    inputs.add_argument('--from-probe',type=Path)
    parser.add_argument('--rectified-input',action='store_true')
    parser.add_argument('--camera-profile')
    parser.add_argument('--handheld',action='store_true')
    parser.add_argument('--urdf',type=Path)
    for name in ('vocabulary', 'run-dir'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--load-atlas', type=Path)
    args = parser.parse_args()
    if args.handheld == (args.urdf is not None):
        parser.error('choose --handheld or --urdf, exactly one')
    calibration = (json.loads(args.calibration.read_text()) if args.calibration else
                   calibration_from_probe(json.loads(args.from_probe.read_text()),args.urdf,args.camera_profile or '',args.rectified_input))
    if args.handheld != (calibration.get('pose_output','base') == 'camera'):
        parser.error('calibration pose_output disagrees with --handheld/--urdf')
    calibration['vocabulary_sha256'] = sha256(args.vocabulary)
    result = prepare_run(args.run_dir, calibration, args.urdf, args.load_atlas)
    print(json.dumps({'status': result['status'], 'run_dir': result['run_dir']}))

if __name__ == '__main__':
    main()
