#!/usr/bin/env python3
# ================================================================
# record_mapping_run.py
# 설명: 지정한 원본 토픽을 MCAP에 기록하고 실제 metadata로 기록 상태를 남긴다.
# 사용법: record_mapping_run.py --run-dir RUN --topics /left /right --required-topics /left /right
# ================================================================
import argparse
import json
import signal
import struct
import subprocess
from pathlib import Path
import yaml
from prepare_mapping_run import atomic_json


def build_record_command(run_dir: Path, topics: list[str], qos_path: Path, storage_path: Path) -> list[str]:
    if not topics or any(not t.startswith('/') or t.startswith('/-') for t in topics):
        raise ValueError('explicit absolute topic names required')
    if (run_dir / 'raw').exists():
        raise FileExistsError(run_dir / 'raw')
    return ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(run_dir/'raw'),
            '--qos-profile-overrides-path', str(qos_path), '--storage-config-file', str(storage_path), *topics]


def finalize_recording(run_dir: Path, return_code: int, topic_counts: dict[str, int], required_topics: list[str] | None = None) -> dict:
    required = required_topics or ['/left', '/right']
    missing = [t for t in required if topic_counts.get(t, 0) <= 0]
    result = {'recording_status': 'failed' if return_code != 0 else ('incomplete' if missing else 'complete'),
              'return_code': return_code, 'topic_counts': topic_counts, 'required_topics': required, 'missing_topics': missing}
    atomic_json(run_dir/'recording.json', result)
    return result


def read_record_counts(metadata: Path) -> dict[str,int]:
    try:
        data=yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
        counts={t['topic_metadata']['name']:int(t['message_count']) for t in data['topics_with_message_count']}
        if any(not isinstance(topic,str) or count<0 for topic,count in counts.items()):
            raise ValueError('invalid topic/count')
        return counts
    except (KeyError,TypeError,ValueError,yaml.YAMLError) as error:
        raise ValueError('invalid rosbag metadata') from error


def validate_mcap_files(directory: Path) -> None:
    """정상 metadata와 별개로 MCAP의 누락·잘린 record·footer를 거부한다(CRC 검사는 별도)."""
    files=list(directory.glob('*.mcap'))
    if not files:
        raise ValueError('MCAP file missing')
    magic=b'\x89MCAP0\r\n'
    for path in files:
        size=path.stat().st_size
        with path.open('rb') as stream:
            if size<58 or stream.read(8)!=magic:
                raise ValueError(f'invalid MCAP header: {path.name}')
            stream.seek(-8,2)
            if stream.read(8)!=magic:
                raise ValueError(f'truncated MCAP tail: {path.name}')
            stream.seek(8)
            saw_data_end=False
            while stream.tell()<size-8:
                offset=stream.tell()
                record=stream.read(9)
                if len(record)!=9:
                    raise ValueError(f'truncated MCAP record: {path.name}')
                opcode,length=struct.unpack('<BQ',record)
                end=offset+9+length
                if end>size-8 or (offset==8 and opcode!=1):
                    raise ValueError(f'invalid MCAP record length/header: {path.name}')
                if opcode==15:
                    saw_data_end=length==4
                if opcode==2:
                    if length!=20 or end!=size-8 or not saw_data_end:
                        raise ValueError(f'invalid MCAP footer: {path.name}')
                    summary,summary_offsets,_crc=struct.unpack('<QQI',stream.read(20))
                    if not (summary==0 or 8<=summary<offset) or not (summary_offsets==0 or summary<=summary_offsets<offset):
                        raise ValueError(f'invalid MCAP summary offsets: {path.name}')
                    break
                stream.seek(end)
            else:
                raise ValueError(f'MCAP footer missing: {path.name}')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--topics', nargs='+', required=True)
    parser.add_argument('--required-topics', nargs='+', required=True)
    parser.add_argument('--qos', type=Path, required=True)
    parser.add_argument('--storage-config', type=Path, required=True)
    args = parser.parse_args()
    if not (args.run_dir/'manifest.json').is_file():
        parser.error('prepare_mapping_run must run first')
    if not set(args.required_topics).issubset(args.topics):
        parser.error('required topics must be included in recorded topics')
    command = build_record_command(args.run_dir, args.topics, args.qos, args.storage_config)
    atomic_json(args.run_dir/'recording.json', {'recording_status': 'running', 'command': command})
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    rclpy.init(); node=rclpy.create_node('mapping_recorder_status')
    publisher=node.create_publisher(String,'/mapping/recording_status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def publish_status(value: dict) -> None:
        publisher.publish(String(data=json.dumps(value)))
        rclpy.spin_once(node,timeout_sec=.01)
    with (args.run_dir/'recorder.log').open('x') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        def stop(_signum: int, _frame: object) -> None:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, stop)
        while process.poll() is None:
            publish_status({'recording_status':'running','run_id':args.run_dir.name})
            rclpy.spin_once(node,timeout_sec=.2)
        return_code = process.wait()
    metadata = args.run_dir/'raw/metadata.yaml'
    counts = {}
    metadata_error=None
    if metadata.is_file():
        try:
            counts=read_record_counts(metadata)
            validate_mcap_files(args.run_dir/'raw')
        except (ValueError,OSError) as error:
            return_code=1; metadata_error=str(error)
    result = finalize_recording(args.run_dir, return_code, counts, args.required_topics)
    if metadata_error:
        result['metadata_error']=metadata_error
        atomic_json(args.run_dir/'recording.json',result)
    publish_status(result)
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    print(json.dumps(result))
    raise SystemExit(0 if result['recording_status'] == 'complete' else 1)

if __name__ == '__main__':
    main()
