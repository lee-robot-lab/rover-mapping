# ================================================================
# test_record_mapping_run.py
# 설명: 실제 토픽의 원본 기록과 비정상 종료/빈 bag을 검증한다.
# 사용법: /usr/bin/python3 -m pytest tests/headless_mapping
# ================================================================
import pytest
from record_mapping_run import build_record_command, finalize_recording

def test_real_time_explicit_topics(tmp_path):
    cmd = build_record_command(tmp_path, ['/left', '/right'], tmp_path/'qos.yaml', tmp_path/'mcap.yaml')
    assert '--use-sim-time' not in cmd and '-a' not in cmd
    assert '/left' in cmd and '/right' in cmd
    assert cmd[cmd.index('-s') + 1] == 'mcap'

@pytest.mark.parametrize('code,counts', [(0, {'/left': 100, '/right': 0}), (1, {'/left': 100, '/right': 100}), (0, {})])
def test_incomplete_or_failure(tmp_path, code, counts):
    result = finalize_recording(tmp_path, code, counts, ['/left', '/right'])
    assert result['recording_status'] != 'complete'

def test_complete_counts_retained(tmp_path):
    result = finalize_recording(tmp_path, 0, {'/left': 2, '/right': 2}, ['/left', '/right'])
    assert result['recording_status'] == 'complete'
    assert result['topic_counts']['/left'] == 2

def test_existing_bag_rejected(tmp_path):
    (tmp_path / 'raw').mkdir()
    with pytest.raises(FileExistsError): build_record_command(tmp_path, ['/left'], tmp_path/'q', tmp_path/'m')

def test_invalid_metadata_rejected(tmp_path):
    from record_mapping_run import read_record_counts
    path=tmp_path/'metadata.yaml'; path.write_text('rosbag2_bagfile_information: {}')
    with pytest.raises(ValueError): read_record_counts(path)

def test_mcap_truncation_rejected_even_with_success_metadata(tmp_path):
    import struct
    from record_mapping_run import validate_mcap_files
    magic=b'\x89MCAP0\r\n'
    header=bytes([1])+struct.pack('<Q',8)+struct.pack('<II',0,0)
    data_end=bytes([15])+struct.pack('<Q',4)+struct.pack('<I',0)
    footer=bytes([2])+struct.pack('<Q',20)+struct.pack('<QQI',0,0,0)
    path=tmp_path/'raw_0.mcap'; path.write_bytes(magic+header+data_end+footer+magic)
    validate_mcap_files(tmp_path)
    path.write_bytes(path.read_bytes()[:-9])
    with pytest.raises(ValueError,match='MCAP'): validate_mcap_files(tmp_path)

def test_mcap_missing_file_rejected(tmp_path):
    from record_mapping_run import validate_mcap_files
    with pytest.raises(ValueError,match='MCAP'): validate_mcap_files(tmp_path)
