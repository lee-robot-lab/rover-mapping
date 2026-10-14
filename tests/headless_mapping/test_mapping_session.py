# ================================================================
# test_mapping_session.py
# 설명: 수동 구간 표시와 종료 리포트가 원본·다중 지도·실패 상태를 정직하게 유지하는지 확인한다.
# 사용법: /usr/bin/python3 -m pytest tests/headless_mapping/test_mapping_session.py
# ================================================================
import json
import pytest


def test_event_records_host_time_without_sensor_time(tmp_path):
    from mapping_session import append_event
    (tmp_path/'manifest.json').write_text('{"status":"prepared"}')
    append_event(tmp_path,'straight','start','manual drive')
    append_event(tmp_path,'straight','end','')
    events=[json.loads(line) for line in (tmp_path/'events.jsonl').read_text().splitlines()]
    assert len(events)==2 and events[0]['run_id']==tmp_path.name
    assert events[0]['clock_source']=='operator_host_receipt'
    assert events[0]['host_ns']>0 and 'sensor_stamp' not in events[0]
    assert events[0]['monotonic_ns']<=events[1]['monotonic_ns']


def test_event_requires_prepared_run(tmp_path):
    from mapping_session import append_event
    with pytest.raises(FileNotFoundError): append_event(tmp_path,'stop','mark','')
    assert not (tmp_path/'events.jsonl').exists()


def test_missing_recording_never_reports_complete(tmp_path):
    from mapping_session import summarize_run
    (tmp_path/'manifest.json').write_text('{"status":"prepared"}')
    (tmp_path/'tracking.json').write_text('{"status":"saved","image_pairs":100,"published_poses":80}')
    result=summarize_run(tmp_path)
    assert result['recording_verified'] is False
    assert result['issues'] and result['published_pose_fraction']==.8
    assert result['atlas_reuse_verified'] is False


def test_reused_manifest_success_cannot_hide_truncated_recording(tmp_path):
    from mapping_session import summarize_run
    (tmp_path/'manifest.json').write_text('{"status":"saved_and_recorded"}')
    (tmp_path/'recording.json').write_text('{"recording_status":"complete","required_topics":["/left","/right"]}')
    raw=tmp_path/'raw'; raw.mkdir(); (raw/'raw_0.mcap').write_bytes(b'broken')
    result=summarize_run(tmp_path)
    assert result['recording_verified'] is False and result['issues']


def test_report_rechecks_input_atlas_checksum(tmp_path):
    from mapping_session import summarize_run
    (tmp_path/'manifest.json').write_text('{"atlas_load_requested":true,"atlas_input_sha256":"previous"}')
    (tmp_path/'atlas_input.osa').write_bytes(b'changed')
    assert any('Atlas input' in issue for issue in summarize_run(tmp_path)['issues'])
