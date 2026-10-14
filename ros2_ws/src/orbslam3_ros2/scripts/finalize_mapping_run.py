#!/usr/bin/env python3
# ================================================================
# finalize_mapping_run.py
# 설명: 종료한 run의 실제 기록·지도 파일을 검사하고 manifest에 단계별 상태를 보존한다.
# 사용법: finalize_mapping_run.py --run-dir RUN
# ================================================================
import argparse
import json
from pathlib import Path
from prepare_mapping_run import atomic_json, sha256


def finalize_run(run_dir: Path) -> dict:
    manifest=json.loads((run_dir/'manifest.json').read_text())
    stages={}
    for name in ('tracking','recording','telemetry'):
        file=run_dir/(name+'.json')
        stages[name]=json.loads(file.read_text()) if file.is_file() else {'status':'not_run_or_interrupted'}
    atlas=run_dir/'atlas.osa'
    files=[p for p in run_dir.rglob('*') if p.is_file() and p.name != 'manifest.json' and not p.name.endswith('.tmp')]
    manifest['files']={str(p.relative_to(run_dir)):{'bytes':p.stat().st_size,'sha256':sha256(p)} for p in files}
    manifest['stages']=stages
    map_present=any(p.name.startswith('map_') and p.suffix=='.ply' and p.stat().st_size>0 for p in files)
    manifest['status']='saved_and_recorded' if (atlas.is_file() and atlas.stat().st_size>0 and map_present and
        stages['tracking'].get('status')=='saved' and stages['recording'].get('recording_status')=='complete') else 'incomplete'
    # Atlas parse/load and separate-drive reuse require actual new-process evidence.
    manifest['atlas_load_verified']=bool(manifest.get('atlas_load_requested') and stages['tracking'].get('atlas_load_verified'))
    manifest.setdefault('atlas_reuse_verified',False)
    if (run_dir/'atlas_input.osa').is_file() and sha256(run_dir/'atlas_input.osa') != manifest.get('atlas_input_sha256'):
        manifest['status']='failed'; manifest['atlas_input_changed']=True
    atomic_json(run_dir/'manifest.json',manifest)
    return manifest


def main() -> None:
    parser=argparse.ArgumentParser(); parser.add_argument('--run-dir',type=Path,required=True)
    result=finalize_run(parser.parse_args().run_dir)
    print(json.dumps({'status':result['status'],'atlas_load_verified':result['atlas_load_verified'],'atlas_reuse_verified':result['atlas_reuse_verified']}))
    raise SystemExit(0 if result['status']=='saved_and_recorded' else 1)

if __name__=='__main__': main()
