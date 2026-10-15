# ================================================================
# offline.py
# 설명: 센서 전송만으로 저장한 원본을 노트북의 native ORB로 재생한다.
# 사용법: .venv-mapping/bin/python -m experiments.handheld_mapping.offline RUN
# ================================================================
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
from typing import Iterator

from experiments.handheld_mapping.compare import ROOT, runner_environment, summarize_timings, vocabularies
from experiments.handheld_mapping.storage import atomic_json


def stream_frames(run: Path, side: str) -> Iterator[tuple[int, str]]:
    """인덱스를 순차 스캔하며 한 stream의 현재 레코드만 유지한다."""
    previous = -1
    with (run/'frames.jsonl').open() as stream:
        for line in stream:
            frame = json.loads(line)
            if frame['stream'] != side:
                continue
            stamp = frame['stamp_ns']
            if not isinstance(stamp, int) or stamp <= previous:
                raise ValueError('invalid/non-increasing stereo stamp')
            previous = stamp
            path = (run/frame['path']).resolve()
            if not path.is_relative_to((run/'raw').resolve()) or any(c in str(path) for c in '\t\r\n'):
                raise ValueError('invalid stereo path')
            yield stamp, str(path)


def iter_pairs(run: Path) -> Iterator[tuple[int, str, str]]:
    """좌우 레코드 두 개만 유지하는 exact timestamp merge를 수행한다."""
    lefts, rights = stream_frames(run, 'left'), stream_frames(run, 'right')
    left, right = next(lefts, None), next(rights, None)
    while left is not None and right is not None:
        if left[0] == right[0]:
            yield left[0], left[1], right[1]
            left, right = next(lefts, None), next(rights, None)
        elif left[0] < right[0]:
            left = next(lefts, None)
        else:
            right = next(rights, None)
    # 매칭 종료 뒤에도 남은 레코드의 시각 오류를 숨기지 않는다.
    for _ in lefts:
        pass
    for _ in rights:
        pass


def write_settings(run: Path, folder: Path) -> Path:
    """동일 실측 보정으로 offline/live point 실행기의 설정을 작성한다."""
    calibration = json.loads((run/'capture_calibration.json').read_text())['stereo']
    specification = importlib.util.spec_from_file_location('rover_prepare', ROOT/'ros2_ws/src/orbslam3_ros2/scripts/prepare_mapping_run.py')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    settings = folder/'settings.yaml'
    settings.write_text(module.orb_yaml(calibration, False)
                        + 'Tracking.StereoRelocalizationWaitSeconds: 15.0\n')
    return settings


def run_point(run: Path) -> dict:
    """기존 Pi 결과를 덮어쓰지 않고 노트북 ORB의 최종 keyframe을 저장한다."""
    run = Path(run).resolve()
    binary = ROOT/'outputs/handheld_mapping/orb3_comparison_build/rover_orb3_compare'
    if not binary.is_file():
        raise FileNotFoundError('노트북 ORB 실행기가 없습니다: tools/build_mapping_baseline.sh')
    folder = run/'orb_offline'
    folder.mkdir(exist_ok=False)
    index = folder/'pairs.tsv'
    paired = 0
    with index.open('w') as stream:
        for stamp, left, right in iter_pairs(run):
            stream.write(f'{stamp}\t{left}\t{right}\n')
            paired += 1
    if not paired:
        raise ValueError('동일 timestamp의 좌우 IR 원본이 없습니다')
    settings = write_settings(run, folder)
    orb_voc, line_voc = vocabularies()
    with (folder/'runner.log').open('wb') as log:
        subprocess.run([str(binary), str(orb_voc), str(line_voc), str(settings), str(index), str(folder)],
                       cwd=folder, env=runner_environment(True), stdout=log, stderr=log, check=True, timeout=3600)
    if (json.loads((folder/'runner_complete.json').read_text()).get('status') != 'complete'
            or not any(folder.glob('map_*_keyframes.csv'))):
        raise RuntimeError('노트북 ORB 최종 keyframe 저장 확인 실패')
    result = {**summarize_timings(folder/'timings.csv'), 'status': 'complete', 'backend': 'orb_offline',
              'paired_frames': paired, 'source': str(run), 'raw_preserved': True,
              'input_calibration_sha256': hashlib.sha256((run/'capture_calibration.json').read_bytes()).hexdigest()}
    atomic_json(folder/'summary.json', result)
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    print(json.dumps(run_point(args.run), indent=2))


if __name__ == '__main__':
    main()
