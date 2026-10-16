# ================================================================
# test_inertial_verified.py
# 설명: 검증된 개별 IMU 입력 경계와 native capability 계약을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_inertial_verified.py
# ================================================================
import json
from pathlib import Path
import subprocess

import numpy as np
import pytest

from experiments.handheld_mapping.gravity import fingerprint
from experiments.handheld_mapping.inertial import prepare_input


@pytest.fixture
def inertial_run(tmp_path):
    run=tmp_path/'run'; run.mkdir()
    for name in ('capture_calibration.json','imu_calibration.json','robot.urdf'):
        (run/name).write_text('fixture source')
    samples=run/'imu_samples.csv'
    samples.write_text('stamp_ns,ax,ay,az,gx,gy,gz\n'+''.join(
        f'{stamp},0,0,9.81,0,0,0\n' for stamp in range(950_000_000,1_050_000_001,10_000_000)))
    profile={'schema':1,'run_id':run.name,'clock_verified':True,'extrinsic_verified':True,
             'sample_timestamps_verified':True,'noise_verified':True,
             'sample_format':'per_sample_SI_in_camera_clock','frame':'rover_body_aligned_imu',
             'acc_unit':'m/s^2','gyro_unit':'rad/s','samples':'imu_samples.csv','frequency_hz':100.,
             'T_imu_left':np.eye(4).tolist(),
             'noise':dict(NoiseGyro=.001,NoiseAcc=.01,GyroWalk=.000001,AccWalk=.0001),
             'source_sha256':{p.name:fingerprint(p) for p in run.iterdir()}}
    path=run/'profile.json'; path.write_text(json.dumps(profile))
    folder=run/'orb_inertial'; folder.mkdir()
    (folder/'pairs.tsv').write_text('1000000000\tleft.png\tright.png\n')
    (folder/'settings.yaml').write_text('%YAML:1.0\n---\nTracking.StereoRelocalizationWaitSeconds: 15.0\n')
    return run,folder,path,profile


def test_verified_si_samples_and_transform_generate_inertial_settings(inertial_run):
    run,folder,path,profile=inertial_run
    result=prepare_input(run,folder,path)
    assert result['sample_count']==11
    assert result['clock_verified'] is True
    text=(folder/'settings.yaml').read_text()
    assert 'Tracking.StereoRelocalizationWaitSeconds' not in text
    assert 'IMU.T_b_c1:' in text and 'IMU.NoiseGyro: 0.001' in text
    assert (folder/'inertial_profile.json').read_bytes()==path.read_bytes()


@pytest.mark.parametrize('key,value',[
    ('run_id','other-run'),('clock_verified',False),('extrinsic_verified',False),
    ('noise_verified',False),('sample_timestamps_verified',False),
    ('sample_format','control_packet_average'),('acc_unit','g'),('gyro_unit','deg/s'),
    ('frame','chip'),('frequency_hz',float('nan')),('T_imu_left',np.diag([2.,1.,1.,1.]).tolist()),
    ('samples','../../other.csv'),('noise',{}),('source_sha256',{}),
])
def test_incomplete_or_mismatched_inertial_contract_refused(inertial_run,key,value):
    run,folder,path,profile=inertial_run
    profile[key]=value; path.write_text(json.dumps(profile))
    with pytest.raises((ValueError,FileNotFoundError)):prepare_input(run,folder,path)
    assert 'IMU.T_b_c1:' not in (folder/'settings.yaml').read_text()


@pytest.mark.parametrize('problem',['duplicate','gap','missing_coverage','nonfinite'])
def test_sample_clock_coverage_and_finiteness_are_checked(inertial_run,problem):
    run,folder,path,profile=inertial_run
    sample=run/'imu_samples.csv'; rows=sample.read_text().splitlines()
    if problem=='duplicate':rows.insert(3,rows[2])
    if problem=='gap':del rows[3:7]
    if problem=='missing_coverage':rows=rows[:4]
    if problem=='nonfinite':rows[3]=rows[3].replace('9.81','nan')
    sample.write_text('\n'.join(rows)+'\n')
    profile['source_sha256']['imu_samples.csv']=fingerprint(sample)
    path.write_text(json.dumps(profile))
    with pytest.raises(ValueError):prepare_input(run,folder,path)


def test_native_runner_rejects_bad_imu_before_slam_starts(tmp_path):
    from experiments.handheld_mapping.compare import ROOT,runner_environment
    binary=ROOT/'outputs/handheld_mapping/orb3_comparison_build/rover_orb3_compare'
    if not binary.is_file():pytest.skip('optional native runner unavailable')
    sample=tmp_path/'bad.csv'; sample.write_text('unverified telemetry\n')
    result=subprocess.run([str(binary),'missing_vocab','unused','missing_yaml','missing_pairs',str(tmp_path),
                           str(sample)],env=runner_environment(True),capture_output=True,text=True,timeout=10)
    if result.returncode==2 and '[imu.csv]' not in result.stderr:pytest.skip('native runner has not been rebuilt for inertial input')
    assert result.returncode==1
    assert 'IMU CSV header does not match SI-unit contract' in result.stderr
    assert 'Vocabulary' not in result.stdout
