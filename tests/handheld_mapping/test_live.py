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


def test_exact_pairing_preserves_consecutive_fifo_and_depth_tolerance(tmp_path):
    from experiments.handheld_mapping.coverage import StereoMailbox
    box = StereoMailbox(tmp_path)
    def offer(side,stamp): box.offer({'stream':side,'stamp_ns':stamp})
    offer('left',100); offer('right',101)
    assert box.take(.001) is None
    for stamp in range(200,210):
        offer('left',stamp); offer('right',stamp); offer('depth',stamp+1)
    assert box.snapshot()['queued_pairs'] == 10
    assert box.snapshot()['dropped_pairs'] == 0
    pair = box.take(.001)
    assert pair['stamp_ns'] == 200
    assert pair['left'] == str(tmp_path/'raw/left/200.png')
    assert box.depth_for(208)['stamp_ns'] == 208
    assert box.depth_for(3_000_000) is None
    box.close()
    assert [box.take(.001)['stamp_ns'] for _ in range(9)] == [201,202,203,204,205,206,207,208,209]
    assert box.take(.001) is None


def test_pair_keeps_oldest_laptop_receipt_so_receive_queue_wait_counts_as_lag(tmp_path):
    from experiments.handheld_mapping.coverage import StereoMailbox
    box = StereoMailbox(tmp_path)
    box.offer({'stream':'left','stamp_ns':5,'laptop_receipt_monotonic_ns':2_000_000_000})
    box.offer({'stream':'right','stamp_ns':5,'laptop_receipt_monotonic_ns':1_000_000_000})
    assert box.take(.001)['offered_at'] == pytest.approx(1.)


def test_fifo_overflow_drops_new_pairs_and_unmatched_metadata_stays_bounded(tmp_path):
    from experiments.handheld_mapping.coverage import StereoMailbox
    box = StereoMailbox(tmp_path)
    for stamp in range(1,261):
        for side in ('left','right'):
            box.offer({'stream':side,'stamp_ns':stamp})
    state = box.snapshot()
    assert state['queued_pairs'] == state['pair_limit'] == 256
    assert state['dropped_pairs'] == 4
    assert state['overflow_policy'] == 'drop_new'
    assert [box.take(.001)['stamp_ns'] for _ in range(256)] == list(range(1,257))
    for stamp in range(1000,1600):
        box.offer({'stream':'left','stamp_ns':stamp})
        box.offer({'stream':'right','stamp_ns':stamp+1000})
        box.offer({'stream':'depth','stamp_ns':stamp})
    state = box.snapshot()
    assert state['unmatched_frames'] == 512
    assert state['unmatched_dropped'] == 688
    assert state['depth_metadata'] == 512
    assert box.depth_for(1599)['stamp_ns'] == 1599


def test_depth_tolerance_includes_boundary_and_excludes_older_metadata(tmp_path):
    from experiments.handheld_mapping.coverage import StereoMailbox
    box = StereoMailbox(tmp_path)
    box.offer({'stream':'depth','stamp_ns':10_000_000})
    assert box.depth_for(12_000_000)['stamp_ns'] == 10_000_000
    assert box.depth_for(12_000_001) is None
    for stamp in range(20_000_000, 20_000_512):
        box.offer({'stream':'depth','stamp_ns':stamp})
    assert box.depth_for(10_000_000) is None


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
        assert state['dropped_pairs'] == 0
    finally: mapper.finish()
    assert mapper.process.poll() == 0
    assert not mapper.thread.is_alive()
    assert mapper.snapshot()['processed'] == 79
    assert mapper.snapshot()['observed_cells'] > 0
    import csv
    with (run/'orb_live/live_poses.csv').open() as source:
        assert [int(row['stamp_ns']) for row in csv.DictReader(source)] == list(range(1,80))


def test_missing_depth_never_delays_ir_tracking_and_finish_drains_fifo(tmp_path):
    import time
    from experiments.handheld_mapping.live import LiveMapper
    run = live_run(tmp_path)
    mapper = LiveMapper(run,command=fake_live_command(run/'orb_live'),result_timeout=2.)
    try:
        for stamp in range(1,21):
            for side in ('left','right'):
                mapper.offer({'stream':side,'stamp_ns':stamp})
        deadline = time.monotonic()+1.
        while mapper.snapshot()['processed'] < 20 and time.monotonic() < deadline:
            time.sleep(.01)
        assert mapper.snapshot()['processed'] == 20, mapper.snapshot()
        assert mapper.snapshot()['no_depth'] == 20
    finally:
        mapper.finish()
    assert mapper.snapshot()['queued_pairs'] == 0
    assert mapper.snapshot()['error'] is None


def test_backlog_tracks_all_ir_and_throttles_visualization(tmp_path):
    from experiments.handheld_mapping.live import LiveMapper
    run = live_run(tmp_path)
    mapper = LiveMapper(run,command=fake_live_command(run/'orb_live',delay=.02),result_timeout=2.)
    try:
        for stamp in range(1,21):
            for side in ('left','right','depth'):
                mapper.offer({'stream':side,'stamp_ns':stamp})
    finally:
        mapper.finish()
    snapshot = mapper.snapshot()
    assert snapshot['error'] is None, snapshot
    assert snapshot['processed'] == 20
    assert 0 < snapshot['skipped_visualizations'] < 19
    assert snapshot['observed_cells'] > 0


def test_sustained_backlog_updates_overlay_and_coverage_without_skipping_ir(tmp_path):
    import cv2
    import time
    from experiments.handheld_mapping.live import LiveMapper
    run = live_run(tmp_path)
    for stamp in range(1,71):
        cv2.imwrite(str(run/f'raw/left/{stamp}.png'),np.full((8,8),stamp,np.uint8))
        if stamp > 1:
            cv2.imwrite(str(run/f'raw/depth/{stamp}.png'),np.full((8,8),1500,np.uint16))
    mapper = LiveMapper(run,command=fake_live_command(run/'orb_live',delay=.03),result_timeout=2.)
    try:
        for side in ('left','right','depth'):
            mapper.offer({'stream':side,'stamp_ns':1})
        deadline = time.monotonic()+2.
        while mapper.snapshot()['processed'] < 1 and time.monotonic() < deadline:
            time.sleep(.005)
        first = mapper.snapshot()
        assert first['overlay_stamp_ns'] == 1
        advances = []
        previous = 1
        for stamp in range(2,71):
            for side in ('left','right','depth'):
                mapper.offer({'stream':side,'stamp_ns':stamp})
            current = mapper.snapshot()
            if current['overlay_stamp_ns'] != previous:
                previous = current['overlay_stamp_ns']
                advances.append((time.monotonic(),previous))
            time.sleep(.01)
        state = mapper.snapshot()
        assert state['queued_pairs'] > 0 and state['dropped_pairs'] == 0
        assert len(advances) >= 2, state
        assert all(stamp > 1 for _,stamp in advances)
        assert all(later[0]-earlier[0] >= .16 for earlier,later in zip(advances,advances[1:]))
        assert state['observed_cells'] > first['observed_cells']
    finally:
        mapper.finish()
    assert mapper.snapshot()['processed'] == 70
    assert mapper.snapshot()['error'] is None


def test_old_queued_input_is_stale_despite_recent_result_and_ui_shows_current_raw(tmp_path):
    import cv2
    import time
    from PyQt5.QtWidgets import QApplication
    from experiments.handheld_mapping.app import MappingWindow
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping.live import LiveMapper
    run = live_run(tmp_path)
    for stamp in range(1,13):
        cv2.imwrite(str(run/f'raw/left/{stamp}.png'),np.full((8,8),50,np.uint8))
    mapper = LiveMapper(run,command=fake_live_command(run/'orb_live',delay=.2),result_timeout=2.)
    window = None
    try:
        for stamp in range(1,13):
            for side in ('left','right','depth'):
                mapper.offer({'stream':side,'stamp_ns':stamp})
        deadline = time.monotonic()+3.
        while mapper.snapshot()['processed'] < 6 and time.monotonic() < deadline:
            time.sleep(.01)
        state = mapper.snapshot()
        assert state['processed'] >= 6 and state['lag_ms'] > 1000, state
        assert time.monotonic()-mapper.last_result_at < .5
        assert state['pose_valid'] is True
        assert state['overlay_original'] is not None
        assert state['stale'] is True
        app = QApplication.instance() or QApplication([])
        session = Session()
        session.preview = np.full((8,8),222,np.uint8)
        session.live = mapper
        window = MappingWindow(session)
        window.show()
        app.processEvents()
        window.refresh()
        image = window.preview.pixmap().toImage()
        assert image.pixelColor(image.width()//2,image.height()//2).red() == 222
    finally:
        if window is not None:
            window.close()
        mapper.finish()


def test_recovery_rows_keep_previous_grid_without_publishing_invalid_pose(tmp_path):
    import sys
    import time
    from experiments.handheld_mapping.live import LiveMapper
    run = live_run(tmp_path)
    code = """import sys
from pathlib import Path
out=Path(sys.argv[1])
with (out/'live_poses.csv').open('w',buffering=1) as f:
 f.write('stamp_ns,state,map_id,process_ms,'+','.join('m%d%d'%(i,j) for i in range(4) for j in range(4))+'\\n')
 for line in sys.stdin:
  stamp=int(line.split('\\t')[0]); state=3 if stamp==2 else 2
  pose=['nan']*16 if state==3 else [str(stamp/10 if (i,j)==(0,3) else int(i==j)) for i in range(4) for j in range(4)]
  f.write(str(stamp)+','+str(state)+',0,1,'+','.join(pose)+'\\n')
"""
    mapper = LiveMapper(run,command=[sys.executable,'-u','-c',code,str(run/'orb_live')],result_timeout=2.)

    def process(stamp: int) -> dict:
        for side in ('left','right','depth'):
            mapper.offer({'stream':side,'stamp_ns':stamp})
        deadline = time.monotonic()+2.
        while mapper.snapshot()['processed'] < stamp and time.monotonic() < deadline:
            time.sleep(.01)
        assert mapper.snapshot()['processed'] == stamp, mapper.snapshot()
        return mapper.snapshot()

    try:
        before = process(1)
        recovering = process(2)
        assert recovering['tracking_state'] == 3
        assert recovering['pose_valid'] is False
        assert recovering['map_id'] == before['map_id'] == 0
        assert recovering['camera'] == before['camera']
        assert recovering['path'] == before['path']
        assert recovering['observed_cells'] == before['observed_cells'] > 0
        assert recovering['overlay'] is None and recovering['overlay_original'] is None
        restored = process(3)
        assert restored['pose_valid'] is True
        assert restored['map_changes'] == 0
        assert restored['path'] == [(.1,0.),(.3,0.)]
    finally:
        mapper.finish()


def test_shutdown_drains_full_bounded_queue_even_when_processing_exceeds_eight_seconds(tmp_path):
    from experiments.handheld_mapping.live import LiveMapper
    run = live_run(tmp_path)
    mapper = LiveMapper(run,command=fake_live_command(run/'orb_live',delay=.035),result_timeout=2.)
    for stamp in range(1,257):
        for side in ('left','right'):
            mapper.offer({'stream':side,'stamp_ns':stamp})
    mapper.finish()
    state = mapper.snapshot()
    assert state['error'] is None, state
    assert state['processed'] == 256
    assert state['dropped_pairs'] == 0 and state['queued_pairs'] == 0
    assert mapper.process.poll() == 0
    assert not mapper.thread.is_alive()


def test_runtime_native_exit_invalidates_current_pose_but_retains_last_known_grid(tmp_path):
    import sys
    import time
    from experiments.handheld_mapping.live import LiveMapper
    run = live_run(tmp_path)
    code = """import sys
from pathlib import Path
with (Path(sys.argv[1])/'live_poses.csv').open('w',buffering=1) as f:
 f.write('stamp_ns,state,map_id,process_ms,'+','.join('m%d%d'%(i,j) for i in range(4) for j in range(4))+'\\n')
 stamp=sys.stdin.readline().split('\\t')[0]
 f.write(stamp+',2,0,1,'+','.join(str(int(i==j)) for i in range(4) for j in range(4))+'\\n')
sys.exit(3)
"""
    mapper = LiveMapper(run,command=[sys.executable,'-u','-c',code,str(run/'orb_live')],result_timeout=2.)
    try:
        for side in ('left','right','depth'):
            mapper.offer({'stream':side,'stamp_ns':1})
        deadline = time.monotonic()+2.
        while not mapper.snapshot()['error'] and time.monotonic() < deadline:
            time.sleep(.01)
        state = mapper.snapshot()
        assert state['error'] and state['processed'] == 1, state
        assert state['pose_valid'] is False
        assert state['camera'] == (0.,0.,0.)
    finally:
        mapper.finish()


def test_map_change_discards_previous_map_overlay_while_new_map_backlog_drains(tmp_path):
    import cv2
    import time
    from experiments.handheld_mapping.live import LiveMapper
    run = live_run(tmp_path)
    cv2.imwrite(str(run/'raw/left/1.png'),np.full((8,8),50,np.uint8))
    mapper = LiveMapper(run,command=fake_live_command(run/'orb_live',delay=.1,map_change=True),result_timeout=2.)

    def wait_processed(count: int) -> dict:
        deadline = time.monotonic()+2.
        while mapper.snapshot()['processed'] < count and time.monotonic() < deadline:
            time.sleep(.005)
        return mapper.snapshot()

    try:
        for side in ('left','right','depth'):
            mapper.offer({'stream':side,'stamp_ns':1})
        first = wait_processed(1)
        assert first['overlay_original'] is not None
        for stamp in (2,3):
            for side in ('left','right','depth'):
                mapper.offer({'stream':side,'stamp_ns':stamp})
        transition = wait_processed(2)
        assert transition['processed'] == 2 and transition['map_id'] == 421
        assert transition['overlay_original'] is None
        assert transition['overlay'] is None
    finally:
        mapper.finish()


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
