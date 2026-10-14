이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# 노트북 지형 지도·costmap 사전 구현 검증 — 2026-10-13

## 범위

실제 Pi/D435는 연결하지 않았다. sensors-only ROS 분기·Wi-Fi 처리율·실측 지형 정확도·경기 STL 위치 보정은 미검증이다. 이 기록은 실제 Python/Qt 전송·후처리 모의 흐름, 알려진 합성 지형, 기존 Isaac Sim stereo의 실제 native ORB/ORB-LINE 실행을 구분한다.

## 확인한 결과

- 관련 회귀 suite: 126 passed. 최종 전체 출력은 `pytest.log`.
- pyflakes/compileall, shell syntax, worker/terrain CLI help 확인.
- 순수 지형: 경사·단차·벽·미관측 영역을 포함한4m×4m/5cm 격자.3 update,18096점 처리. 지속 통계 배열179200B, NPZ 약25KiB. unknown703/blocked1186/valid4511셀. `synthetic_terrain/`에 지형 층·cost·metadata·그림 보존.
- 실제 Qt 모의 촬영: `app_capture/20261013_132345_da9627`, IR좌/우/depth 각22프레임, TSDF 저장 완료. 알려진 합성 pose이며 실제 SLAM 정확도 시험이 아니다.
- 위 촬영의 depth/최종 pose를 새 스트리밍 지형 경로로 재생: `recorded_terrain/`,22 keyframe 모두 융합,105600점,unknown5077/blocked192/valid1131셀. 명시적 합성 중력 정렬은 `demo_alignment.json`에 보존.
- 10m×10m/5cm 격자의 지속 통계 배열은1,120,000B(약1.07MiB). 이 값은 Python RSS·파생 층·한 프레임·native SLAM/TSDF를 포함하지 않는다.

## 실제 native 재생

입력은 기존 `20261013_app_validation/stereo_sim`의180 stereo쌍이다. 보정값과 인덱스를 새 검증 폴더에 복사하고 `raw`는 읽기용 symlink로 재사용했다. 입력에 GT/depth를 주지 않았다.

| 항목 | ORB 점 | ORB-LINE 점+선 |
|---|---:|---:|
| 처리 stereo | 180 | 180 |
| tracking OK | 100% | 100% |
| map | 0 | 0 |
| 최종 keyframe | 38 | 48 |
| tracking 중앙값/p95 | 10.31/11.79ms | 26.36/30.63ms |

점은 새 `offline.run_point`를 실제 실행했다. 점선은 새 스트리밍 pairs 인덱스와 기존 점 결과 재사용 경로를 실제 실행했다. timing 통계는 임시 SQLite 인덱스로 계산한다. 점 O3/native와 선 O1/native의 빌드 차이가 있으며 이 수치는 Pi FPS, 표시 지연, 실기 정확도 또는 알고리즘 우위의 근거가 아니다.

`/usr/bin/time -v` 기록: 점 실행 전체 최대 RSS552844KiB, 점선899748KiB, 합성 지형+그림190168KiB. 실행 전체 소요는 각각7.39초/12.50초/1.45초이며 native 시작·vocabulary 로딩·저장·그림 등이 포함된다. 큐/격자 배열 상한과 프로세스 전체 RSS를 구분한다.

## 최종 검토에서 수정한 사항

1. native keyframe CSV가 timestamp 순서라는 보장이 없었다. 실제38 keyframe에서 순서 역행을 확인했다. 소유 임시 SQLite 인덱스의2MiB page cache로 정렬하고 duplicate stamp/map 불일치를 거부한다. `index_validation.json`에서 실제 CSV38개 정렬과 원본 무변경 확인.
2. 기존 TSDF와 timing 요약이 index 전체를 메모리에 올렸다. depth 순차 스캔과 임시 디스크 정렬/정확 분위수 계산으로 보완했다. 선택 점선 비교의 pairs 생성·동일 입력 검증도 streaming/hash로 변경했다.

각 결함의 회귀 시험이 먼저 실패한 뒤 수정 후 통과했다. 최종 독립 리뷰의 추가 Critical/Minor 항목은 없었다. 모든 변경 후 전체 관련 suite와 정적 검사를 수행했다.

## 재현

```bash
cd /home/su/rover
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 \
  .venv-mapping/bin/python -m pytest tests/handheld_mapping tests/headless_mapping -q
bash tools/terrain_mapping.sh --demo --output /absolute/path/to/NEW_OUTPUT
```

`native_replay/orb_offline`과`orbline`은 기존 결과를 덮어쓰지 않는다. 재현 시 새 폴더에 입력 metadata와 raw symlink를 준비한다.

사용 안내: [노트북 지형·costmap 준비](../../experiments/laptop-terrain-preparation.md)
