# ================================================================
# test_estimator_log.py
# 설명: 추정기 입력 기록이 파일을 한 번만 열고, 직렬화 실패로 시뮬 run을 중단시키지 않는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_estimator_log.py -q
# ================================================================
import json
from pathlib import Path

from experiments.site_sim.estimator_log import EstimatorLog


def test_events_are_written_in_order_through_one_handle(tmp_path: Path) -> None:
    log = EstimatorLog(tmp_path)
    for k in range(3):
        log.write({'kind': 'imu', 'stamp_s': k*.1})
    handle = log.stream
    log.write({'kind': 'pose', 'stamp_s': .3})
    assert log.stream is handle
    log.close()
    lines = [json.loads(line) for line in (tmp_path/'estimator_events.jsonl').read_text().splitlines()]
    assert [e['stamp_s'] for e in lines] == [0., .1, .2, .3]


def test_nan_or_closed_file_only_disables_logging_with_a_warning(tmp_path: Path, capsys) -> None:
    log = EstimatorLog(tmp_path)
    log.write({'kind': 'imu', 'stamp_s': 0.})
    log.write({'kind': 'imu', 'stamp_s': float('nan')})
    log.write({'kind': 'imu', 'stamp_s': .2})            # 이미 비활성: 예외 없이 무시
    log.close()
    assert log.failed and 'estimator log disabled' in capsys.readouterr().out
    assert len((tmp_path/'estimator_events.jsonl').read_text().splitlines()) == 1
