#!/usr/bin/env python3
# ================================================================
# mapping_session.py
# 설명: 수동 주행 구간을 host 시각으로 기록하고 종료 자료를 읽어 검증 리포트를 만든다.
# 사용법: mapping_session.py event --run-dir RUN --phase straight --edge start
#   mapping_session.py report --run-dir RUN
# ================================================================
import argparse
import json
import os
from pathlib import Path
import time
from prepare_mapping_run import atomic_json, sha256
from record_mapping_run import read_record_counts, validate_mcap_files


def append_event(run: Path, phase: str, edge: str, note: str) -> dict:
    json.loads((run/'manifest.json').read_text())
    if phase not in ('stationary','straight','turn','return','stop','note') or edge not in ('start','end','mark'):
        raise ValueError('unsupported phase/edge')
    event={'run_id':run.name,'phase':phase,'edge':edge,'note':note,
           'host_ns':time.time_ns(),'monotonic_ns':time.monotonic_ns(),'clock_source':'operator_host_receipt'}
    data=(json.dumps(event,allow_nan=False)+'\n').encode()
    descriptor=os.open(run/'events.jsonl',os.O_WRONLY|os.O_CREAT|os.O_APPEND,0o644)
    try:
        if os.write(descriptor,data)!=len(data): raise OSError('incomplete event write')
    finally: os.close(descriptor)
    return event


def summarize_run(run: Path) -> dict:
    manifest=json.loads((run/'manifest.json').read_text()); stages={}; issues=[]
    for stage in ('tracking','recording','telemetry'):
        file=run/(stage+'.json')
        stages[stage]=json.loads(file.read_text()) if file.is_file() else {}
    tracking=stages['tracking']; recording=stages['recording']; verified=False; counts={}
    try:
        validate_mcap_files(run/'raw')
        counts=read_record_counts(run/'raw/metadata.yaml')
        required=recording['required_topics']
        if not required or any(counts.get(topic,0)<=0 for topic in required):
            raise ValueError('required recorded topics missing or empty')
        if recording.get('recording_status')!='complete' or counts!=recording.get('topic_counts'):
            raise ValueError('recorder status/metadata counts disagree')
        verified=True
    except (OSError,ValueError,KeyError,TypeError) as error:
        issues.append(f'recording: {error}')
    if tracking.get('status')!='saved': issues.append('tracking not saved')
    if not (run/'atlas.osa').is_file() or (run/'atlas.osa').stat().st_size==0:
        issues.append('nonempty Atlas missing')
    if not any(path.stat().st_size for path in run.glob('map_*.ply')): issues.append('nonempty map export missing')
    if manifest.get('atlas_load_requested'):
        source=run/'atlas_input.osa'
        if not source.is_file() or sha256(source)!=manifest.get('atlas_input_sha256'):
            issues.append('Atlas input missing or checksum changed')
    events=[]
    if (run/'events.jsonl').is_file():
        for line in (run/'events.jsonl').read_text().splitlines():
            try:
                event=json.loads(line)
                if event.get('run_id')!=run.name: raise ValueError('event run_id mismatch')
                events.append(event)
            except (ValueError,TypeError,AttributeError) as error: issues.append(f'event: {error}')
    pairs=tracking.get('image_pairs',0); calibration=manifest.get('calibration',{})
    return {'run_id':run.name,'manifest_status':manifest.get('status'),'evidence_complete':not issues,
            'recording_verified':verified,'recorded_topic_counts':counts,'issues':issues,'events':events,
            'tracking':tracking,'published_pose_fraction':tracking.get('published_poses',0)/pairs if pairs>0 else None,
            'map_frames':sorted(path.stem.replace('base_map_','orb_map_') for path in run.glob('base_map_*.tum')),
            'pitch_down_deg':calibration.get('pitch_down_deg'),'extrinsic_verified':calibration.get('extrinsic_verified',False),
            'atlas_load_verified':tracking.get('atlas_load_verified',False),
            'atlas_reuse_verified':manifest.get('atlas_reuse_verified',False),
            'telemetry_status':stages['telemetry'].get('status','not_recorded'),
            'accuracy_evaluated':False,'note':'No GT accuracy or separate-drive reuse inferred; MCAP structure check excludes payload CRC'}


def main() -> None:
    parser=argparse.ArgumentParser(); sub=parser.add_subparsers(dest='command',required=True)
    event=sub.add_parser('event'); report=sub.add_parser('report')
    for command in (event,report): command.add_argument('--run-dir',type=Path,required=True)
    event.add_argument('--phase',choices=['stationary','straight','turn','return','stop','note'],required=True)
    event.add_argument('--edge',choices=['start','end','mark'],default='mark')
    event.add_argument('--note',default='')
    report.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.command=='event': result=append_event(args.run_dir,args.phase,args.edge,args.note)
    else:
        if args.output and args.output.exists(): parser.error('output exists; use a new path')
        result=summarize_run(args.run_dir)
        if args.output: atomic_json(args.output,result)
    print(json.dumps(result,indent=2,allow_nan=False))
    raise SystemExit(0 if args.command=='event' or result['evidence_complete'] else 1)


if __name__=='__main__': main()
