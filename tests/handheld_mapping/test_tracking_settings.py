# ================================================================
# test_tracking_settings.py
# 설명: 노트북 추적기가 실제 읽는 설정에 stereo 복구 대기가 전달되는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_tracking_settings.py -q
# ================================================================
import json
from pathlib import Path

import cv2
import numpy as np

from experiments.handheld_mapping.offline import write_settings


def test_laptop_settings_enable_fifteen_second_stereo_recovery(tmp_path: Path) -> None:
    info = {'width': 640, 'height': 480, 'k': [377., 0, 320., 0, 377., 240., 0, 0, 1], 'd': [0.] * 5}
    stereo = np.eye(4)
    stereo[0, 3] = .05
    calibration = {'pose_output': 'camera', 'camera_profile': '640x480x15:Y8',
                   'left': info, 'right': info, 'T_c1_c2': stereo.tolist()}
    (tmp_path / 'capture_calibration.json').write_text(json.dumps({'stereo': calibration}))
    settings = write_settings(tmp_path, tmp_path)
    consumer = cv2.FileStorage(str(settings), cv2.FILE_STORAGE_READ)
    try:
        assert consumer.getNode('Tracking.StereoRelocalizationWaitSeconds').real() == 15.
        assert consumer.getNode('Camera.fps').real() == 15.
        assert consumer.getNode('Stereo.T_c1_c2').mat()[0, 3] == np.float32(.05)
    finally:
        consumer.release()
