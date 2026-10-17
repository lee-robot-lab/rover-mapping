# ================================================================
# stereo_capture.py
# 설명: 한 render step의 좌우 원본 PNG·simulation timestamp·보정과 별도 GT를 기록한다.
# 사용법: from experiments.site_sim.stereo_capture import StereoCapture
# ================================================================
from pathlib import Path
import csv
import json
import numpy as np
from PIL import Image


class StereoCapture:
    """완성된 좌우 쌍만 EuRoC index에 기록하는 시뮬레이션 평가용 writer."""

    def __init__(self, output: Path, calibration: dict) -> None:
        self.output=Path(output);self.output.mkdir(parents=True,exist_ok=False)
        self.width,self.height=int(calibration['width']),int(calibration['height'])
        self.last=-1
        (self.output/'calibration.json').write_text(json.dumps(calibration,indent=2,allow_nan=False))
        for side in ('cam0','cam1'):
            root=self.output/'mav0'/side;(root/'data').mkdir(parents=True)
            (root/'data.csv').write_text('#timestamp [ns],filename\n')

    def write(self, stamp_ns: int, left: np.ndarray, right: np.ndarray, position: list, quaternion: list) -> None:
        """같은 시각의 pair와 별도 GT를 기록하며 잘못된 입력은 index 갱신 전에 거부한다."""
        if type(stamp_ns) is not int or stamp_ns<=self.last:raise ValueError('strictly increasing integer timestamp required')
        images=[np.asarray(a) for a in (left,right)]
        if any(a.dtype!=np.uint8 or a.shape!=(self.height,self.width,3) for a in images):raise ValueError('stereo RGB image shape/type mismatch')
        if np.asarray(position).shape!=(3,) or np.asarray(quaternion).shape!=(4,) or not np.isfinite([*position,*quaternion]).all():raise ValueError('finite GT required')
        filename=f'{stamp_ns}.png'
        for side,im in zip(('cam0','cam1'),images):
            root=self.output/'mav0'/side;target=root/'data'/filename;tmp=target.with_suffix('.tmp')
            Image.fromarray(im).save(tmp,format='PNG');tmp.replace(target)
        for side in ('cam0','cam1'):
            with (self.output/'mav0'/side/'data.csv').open('a') as f:csv.writer(f).writerow([stamp_ns,filename])
        with (self.output/'gt.jsonl').open('a') as f:
            f.write(json.dumps({'stamp_ns':stamp_ns,'position_m':list(position),'quaternion_wxyz':list(quaternion),'pose_source':'sim_ground_truth'},allow_nan=False)+'\n')
        self.last=stamp_ns
