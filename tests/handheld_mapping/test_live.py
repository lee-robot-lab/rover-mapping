# ================================================================
# test_live.py
# 설명: 실시간 관측 범위의 좌표·시점 중복·지도 분리와 bounded stereo 전달을 검증한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_live.py -q
# ================================================================
import numpy as np
import pytest
from pathlib import Path


def depth_info():
    return {'width': 8, 'height': 8, 'k': [8.,0,3.5,0,8.,3.5,0,0,1],
            'depth_scale_m': .001, 'T_left_depth': np.eye(4).tolist()}


def test_stationary_frames_do_not_inflate_overlap_and_map_change_clears():
    from experiments.handheld_mapping.coverage import CoverageGrid
    grid = CoverageGrid(resolution=.25)
    depth = np.full((8,8), 2000, np.uint16)
    pose = np.eye(4)
    for _ in range(20): grid.update(0, pose, depth, depth_info(), stride=1)
    first = grid.snapshot()
    assert first['observed_cells'] > 0
    assert first['max_observations'] == 1
    pose[1,3] = .5  # 투영 수직 이동도 다른 카메라 위치지만 X-Z가 같으면 중복으로 보수적으로 처리
    grid.update(0, pose, depth, depth_info(), stride=1)
    assert grid.snapshot()['max_observations'] == 1
    pose[0,3] = .12
    grid.update(0, pose, depth, depth_info(), stride=1)
    assert grid.snapshot()['max_observations'] == 2
    grid.update(421, np.eye(4), depth, depth_info(), stride=1)
    assert grid.snapshot()['map_id'] == 421
    assert grid.snapshot()['max_observations'] == 1
    assert len(grid.snapshot()['path']) == 1


def test_depth_extrinsic_rotation_and_unknown_are_preserved():
    from experiments.handheld_mapping.coverage import CoverageGrid
    grid = CoverageGrid(bounds=(-4,4,-4,4), resolution=.1)
    info = depth_info()
    info['T_left_depth'][0][3] = .3
    pose = np.array([[0.,0,1,1],[0,1,0,0],[-1,0,0,0],[0,0,0,1]])
    grid.update(0, pose, np.full((8,8),2000,np.uint16), info, stride=1)
    rows, cols = np.nonzero(grid.counts)
    assert np.median(-4+(cols+.5)*.1) == pytest.approx(3.05,abs=.1)
    assert np.median(-4+(rows+.5)*.1) == pytest.approx(-.3,abs=.15)
    assert grid.counts[0,0] == 0
    before = grid.counts.copy()
    grid.update(0, pose, np.zeros((8,8),np.uint16), info, stride=1)
    assert np.array_equal(before,grid.counts)
    pose[0,3] = 100
    grid.update(0, pose, np.full((8,8),2000,np.uint16),info,stride=1)
    assert grid.snapshot()['outside_points'] > 0
    assert np.array_equal(before,grid.counts)


def test_invalid_pose_and_grid_limits_are_rejected():
    from experiments.handheld_mapping.coverage import CoverageGrid
    with pytest.raises(ValueError): CoverageGrid(resolution=.001)
    grid = CoverageGrid()
    pose = np.eye(4); pose[0,0] = float('nan')
    with pytest.raises(ValueError): grid.update(0,pose,np.zeros((8,8),np.uint16),depth_info())
    assert grid.snapshot()['observed_cells'] == 0


def test_exact_pairing_bounded_latest_queue_and_depth_tolerance(tmp_path):
    from experiments.handheld_mapping.coverage import StereoMailbox
    box = StereoMailbox(tmp_path)
    def offer(side,stamp): box.offer({'stream':side,'stamp_ns':stamp})
    offer('left',100); offer('right',101)
    assert box.take(.001) is None
    for stamp in range(200,210):
        offer('left',stamp); offer('right',stamp); offer('depth',stamp+1)
    assert box.snapshot()['queued_pairs'] == 2
    assert box.snapshot()['dropped_pairs'] == 8
    pair = box.take(.001)
    assert pair['stamp_ns'] == 208
    assert pair['left'] == str(tmp_path/'raw/left/208.png')
    assert box.depth_for(208)['stamp_ns'] == 208
    assert box.depth_for(3_000_000) is None
    for stamp in range(1000,1100): offer('left',stamp); offer('depth',stamp)
    assert box.snapshot()['unmatched_frames'] <= 16
    assert box.snapshot()['depth_metadata'] == 32
    box.close()
    assert box.take(.001)['stamp_ns'] == 209
    assert box.take(.001) is None


def fake_live_command(folder, delay=0., die=False, map_change=False):
    import sys
    code = '''import sys,time
from pathlib import Path
out=Path(sys.argv[1]); out.mkdir(exist_ok=True)
if sys.argv[3]=='True':sys.exit(3)
with (out/'live_poses.csv').open('w',buffering=1) as f:
 f.write('stamp_ns,state,map_id,process_ms,'+','.join('m%d%d'%(i,j) for i in range(4) for j in range(4))+'\\n')
 for line in sys.stdin:
  time.sleep(float(sys.argv[2]))
  stamp=line.split('\\t')[0]
  map_id='421' if sys.argv[4]=='True' and int(stamp)>1 else '0'
  f.write(stamp+',2,'+map_id+',1,'+','.join(str(int(i==j)) for i in range(4) for j in range(4))+'\\n')
(out/'runner_complete.json').write_text('{"status":"complete"}')
'''
    return [sys.executable,'-u','-c',code,str(folder),str(delay),str(die),str(map_change)]


def live_run(tmp_path):
    import json
    import cv2
    run = tmp_path/'run'; run.mkdir()
    (run/'capture_calibration.json').write_text(json.dumps({'depth':depth_info()}))
    for side in ('left','right','depth'): (run/'raw'/side).mkdir(parents=True)
    for stamp in range(1,80):
        cv2.imwrite(str(run/f'raw/depth/{stamp}.png'),np.full((8,8),2000,np.uint16))
    return run


def test_live_result_arrives_before_eof_and_slow_native_does_not_block_offers(tmp_path):
    import time
    from experiments.handheld_mapping.live import LiveMapper
    run = live_run(tmp_path)
    mapper = LiveMapper(run,command=fake_live_command(run/'orb_live',delay=.03),result_timeout=2.)
    try:
        started = time.monotonic()
        for stamp in range(1,80):
            for side in ('left','right','depth'): mapper.offer({'stream':side,'stamp_ns':stamp})
        assert time.monotonic()-started < .15
        deadline = time.monotonic()+3
        while mapper.snapshot()['processed'] == 0 and time.monotonic()<deadline: time.sleep(.01)
        state = mapper.snapshot()
        assert state['processed'] > 0 and mapper.process.poll() is None
        assert state['dropped_pairs'] > 0
        assert state['observed_cells'] > 0
    finally: mapper.finish()
    assert mapper.process.poll() == 0
    assert not mapper.thread.is_alive()


def test_failed_native_is_reported_and_finish_does_not_leave_child(tmp_path):
    import time
    from experiments.handheld_mapping.live import LiveMapper
    run = live_run(tmp_path)
    mapper = LiveMapper(run,command=fake_live_command(run/'orb_live',die=True),result_timeout=.2)
    for side in ('left','right','depth'):mapper.offer({'stream':side,'stamp_ns':1})
    deadline = time.monotonic()+2
    while not mapper.snapshot()['error'] and time.monotonic()<deadline:time.sleep(.01)
    assert mapper.snapshot()['error']
    mapper.finish()
    assert mapper.process.poll() is not None
    assert not mapper.thread.is_alive()


def test_pose_without_depth_updates_camera_but_does_not_mark_observed():
    from experiments.handheld_mapping.coverage import CoverageGrid
    grid=CoverageGrid()
    pose=np.eye(4); pose[0,3]=1.
    grid.update_pose(0,pose)
    snapshot=grid.snapshot()
    assert snapshot['camera'][:2] == (1.,0.)
    assert snapshot['path'] == [(1.,0.)]
    assert snapshot['observed_cells']==0


def test_native_parent_exit_also_cleans_owned_descendant(tmp_path):
    import os
    import signal
    import sys
    import time
    from experiments.handheld_mapping.live import LiveMapper
    run=live_run(tmp_path)
    pidfile=run/'child.pid'
    ready=run/'child.ready'
    child_code='import signal,time,pathlib; signal.signal(signal.SIGTERM,signal.SIG_IGN); pathlib.Path('+repr(str(ready))+').touch(); time.sleep(60)'
    parent_code='import subprocess,sys,time,pathlib; p=subprocess.Popen([sys.executable,"-c",'+repr(child_code)+']); pathlib.Path('+repr(str(pidfile))+').write_text(str(p.pid));\nwhile not pathlib.Path('+repr(str(ready))+').exists():time.sleep(.01)\nsys.exit(3)'
    mapper=LiveMapper(run,command=[sys.executable,'-c',parent_code],result_timeout=.2)
    child=None
    try:
        deadline=time.monotonic()+3
        while not pidfile.exists() and time.monotonic()<deadline:time.sleep(.01)
        child=int(pidfile.read_text())
        mapper.finish()
        stat=Path(f'/proc/{child}/stat')
        assert not stat.exists() or stat.read_text().split()[2]=='Z', 'owned descendant is still running'
    finally:
        mapper.finish()
        if child is not None:
            try:os.kill(child,signal.SIGKILL)
            except ProcessLookupError:pass


def test_map_change_archives_previous_coverage_and_resets_active_grid(tmp_path):
    import time
    from experiments.handheld_mapping.live import LiveMapper
    run=live_run(tmp_path)
    mapper=LiveMapper(run,command=fake_live_command(run/'orb_live',map_change=True),result_timeout=2.)
    try:
        for stamp in (1,2):
            for side in ('left','right','depth'):mapper.offer({'stream':side,'stamp_ns':stamp})
            deadline=time.monotonic()+2
            while mapper.snapshot()['processed']<stamp and time.monotonic()<deadline:time.sleep(.01)
        snapshot=mapper.snapshot()
        assert snapshot['map_id']==421 and snapshot['map_changes']==1
        with np.load(run/'orb_live/coverage_map_0.npz',allow_pickle=False) as archive:
            assert np.count_nonzero(archive['observations'])>0
        assert snapshot['max_observations']==1
    finally:mapper.finish()
    assert (run/'orb_live/coverage_map_421.npz').is_file()
