# ================================================================
# test_display.py
# 설명: 읽기 전용 상태 화면이 오래된 상태를 준비 완료로 오인하지 않는지 검증한다.
# 사용법: python -m pytest tests/pi_navigation/test_display.py -q
# ================================================================
import json
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'deployment/ksrc'))


def write_status(state: Path, **changes: object) -> None:
    status = dict(active=False, recovering=False, done=False, run=0, stage='entry30', reason='waiting',
                  orb_ok=True, stationary_s=4., in_region1=True, start_aligned=True,
                  command=[0., 0., 0.], pose_age_s=.05, imu_age_s=.01, recovery_count=0)
    status.update(changes)
    (state / 'status.json').write_text(json.dumps(status))
    (state / 'fake.json').write_text('{}')


def test_ready_view_explains_fake_and_no_mission(tmp_path):
    from display import view_text
    write_status(tmp_path)
    out = view_text(tmp_path)
    assert 'FAKE' in out and 'run=0' in out and 'START 가능' in out
    assert '실제 맵 일치' in out
    assert '3분' not in out


@pytest.mark.parametrize('stopped', [False, True])
def test_old_or_stopped_status_is_never_ready(tmp_path, stopped):
    from display import view_text
    write_status(tmp_path)
    if stopped:
        (tmp_path / 'stopped.json').write_text('{}')
    else:
        os.utime(tmp_path / 'status.json', (1., 1.))
    out = view_text(tmp_path)
    assert 'START 가능' not in out
    assert ('종료됨' if stopped else '오래된 상태') in out


def test_auto_view_does_not_treat_leaving_start_as_failure(tmp_path):
    from display import view_text
    write_status(tmp_path, active=True, run=2, stage='return1', in_region1=False, start_aligned=False)
    out = view_text(tmp_path)
    assert 'AUTO' in out and 'run=2' in out and 'return1' in out
    assert '출발 조건은 재평가하지 않음' in out
    assert 'START 가능' not in out


def test_missing_view_does_not_create_files(tmp_path):
    from display import view_text
    assert '초기화' in view_text(tmp_path)
    assert not list(tmp_path.iterdir())


def test_report_excludes_boot_average_and_post_stop_load(tmp_path):
    from display import report_text
    import datetime
    import numpy as np
    load = tmp_path / 'load'
    load.mkdir()
    (load / 'meta.txt').write_text('2026-10-22T12:00:00+09:00\n')
    (load / 'top_threads.log').write_text(
        'top - 12:00:00 up\n%Cpu0 : 0 us, 0 id\n'
        'top - 12:00:01 up\n%Cpu0 : 10 us, 90 id\n'
        'top - 12:00:02 up\n%Cpu0 : 40 us, 60 id\n'
        'top - 12:00:03 up\n%Cpu0 : 99 us, 1 id\n')
    cutoff = datetime.datetime.fromisoformat('2026-10-22T12:00:02.500000+09:00').timestamp()
    (tmp_path / 'stopped.json').write_text(json.dumps(dict(time=cutoff)))
    np.savez(tmp_path / 'policy.npz', period_ms=[20., 20., 30.], compute_ms=[2., 3., 4.])
    out = report_text(tmp_path)
    assert 'CPU0: 평균 25.0% / 최대 40.0%' in out
    assert '25 ms 초과 1/3' in out
    assert '수집 중' in out  # finished.txt가 없어도 정책 저장과 별개다.


def test_failed_start_by_global_lock_is_not_shown_as_unknown_mode(tmp_path):
    from display import view_text
    (tmp_path / 'supervisor.log').write_text('Traceback ...\nBlockingIOError: [Errno 11] Resource temporarily unavailable\n')
    out = view_text(tmp_path)
    assert '시작 실패' in out and '모드 미확인' not in out and '_supervise' in out
