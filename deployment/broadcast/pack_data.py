# ================================================================
# pack_data.py
# 설명: 중계 캐시를 16개 표시 후보·float16 좌표로 줄여 Git 배포 크기로 저장하고 오차를 검증한다.
# 사용법: python deployment/broadcast/pack_data.py --source /path/to/transfer_v1
# ================================================================
from __future__ import annotations
import argparse
import json
from pathlib import Path
import shutil
import numpy as np


def main() -> None:
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,required=True)
    args=parser.parse_args();out=Path(__file__).with_name('data');out.mkdir(exist_ok=True)
    errors={}
    for filename in ('predictions.npz','teleop_cache.npz'):
        with np.load(args.source/filename) as source:
            arrays={key:source[key] for key in source.files}
        arrays['candidates']=arrays['candidates'][:,::2]
        arrays['costs']=arrays['costs'][:,::2]
        for key in ('candidates','chosen'):
            original=arrays[key];packed=original.astype(np.float16)
            error=float(np.max(np.abs(original-packed.astype(original.dtype))))
            assert error<.0041,(filename,key,error)
            errors[filename+'/'+key]=error;arrays[key]=packed
        np.savez_compressed(out/filename,**arrays)
        assert (out/filename).stat().st_size<95*1024**2, 'asset exceeds Git per-file size budget'
        print(filename,round((out/filename).stat().st_size/1024**2,1),'MiB',flush=True)
    for filename in ('terrain.npz','terrain.png','manifest.json','basket_start.json'):
        path=args.source/filename
        if path.exists():shutil.copy2(path,out/filename)
    manifest=json.loads((out/'manifest.json').read_text())
    manifest['deployment_pack']=dict(display_candidates=16,coordinate_dtype='float16',max_coordinate_error_m=max(errors.values()))
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    (out/'packing_verification.json').write_text(json.dumps(errors,indent=2))
    print('max coordinate error m',max(errors.values()))


if __name__=='__main__':main()
