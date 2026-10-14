이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../../data-assets.md)를 따른다.

# ORB 원본 재생 조사 결과

동일 원본·timestamp·설정에서도 159–164초 구간의 tracking loss와 맵 분리는 반복됐지만, 정확한 실패 프레임·맵 수는 실행마다 달랐다. `nFeatures`만 1200→2400으로 늘려도 주 실패 구간이 유지됐고 처리시간이 늘었다. 2400 첫 실행은 `Shutdown` 뒤 SIGSEGV로 종료돼 최종 저장에 실패했다. 이 실험만으로 production 설정을 변경할 근거는 없다.

## 실행 범위

- 원본: `/home/su/rover/outputs/handheld_mapping/sessions/20261013_204443_766235/orb_offline/pairs.tsv`의 3604 stereo 쌍.
- baseline은 원본 settings.yaml을 직접 읽었다. 2400은 `ORBextractor.nFeatures` 한 줄만 바꾼 별도 settings.yaml을 읽었다.
- 모든 실행에서 원본 image 경로와 timestamp를 그대로 사용했고 native 재생은 순차 실행했다. 원본 파일·production/native source를 쓰지 않았다. CLAHE·시간 보정·pose 합성·map merge는 하지 않았다.
- Python 3.10.12 `.venv-mapping`, `PYTHONNOUSERSITE=1`, `OMP_NUM_THREADS=2`, `OPENBLAS_NUM_THREADS=1`, 기존 `runner_environment(True)`·`vocabularies()`를 사용했다.
- 최초 저장 결과의 실행 환경과 당시 runtime hash는 기록돼 있지 않다. 현재 두 baseline 재생끼리는 같은 binary/runtime/environment를 사용했고 서로 결과가 달랐다.

## 결과표

| 실행 | 특징 수 | 비정상/전체 frame | 관측 map ID | 최종 저장 | keyframe 수 | tracking ms 중앙값 / P95 |
|---|---:|---:|---|---|---:|---:|
| 기존 session 결과 | 1200 | 45 / 3604 | 0, 421 | complete | 815 | 16.35 / 19.39 |
| baseline_repeat | 1200 | 76 / 3604 | 0, 407, 763 | exit 0, complete | 782 | 16.78 / 19.78 |
| baseline_repeat2 | 1200 | 46 / 3604 | 0, 413 | exit 0, complete | 788 | 16.54 / 20.28 |
| features2400 | 2400 | 45 / 3604 | 0, 247 | **SIGSEGV -11, incomplete** | 미저장 | 23.17 / 29.33 |
| features2400_repeat_gdb | 2400 | 46 / 3604 | 0, 241 | exit 0, complete | 459 | 23.37 / 30.28 |

비정상은 `state != 2`이다. `map_id`는 Atlas의 map ordinal이 아니라 첫 keyframe ID(`GetInitKFid()`)다. 실패한 2400 실행의 map ID는 추적 중 관측값이며 최종 저장된 맵으로 집계하지 않는다. GDB 반복은 관찰 방식이 추가돼 scheduling 영향이 있을 수 있다. 표의 ms는 offline `TrackStereo` 호출 시간이며 Pi FPS·live latency·GT 정확도가 아니다.

### 실패 구간

- 기존 결과: 160.616112793–163.679791992초, 45 frame. 첫 실패 직전 gap은 0.932429932초.
- baseline_repeat: 159.417277832–162.480957275초, 29 frame; 230.870772217–233.934017822초, 47 frame. 첫 구간 시작 gap은 0.266404053초이고 두 번째는 정상 주기인 0.066592041초였다.
- baseline_repeat2: 기존과 같은 160.616112793–163.679791992초 45 frame; 230.870772217초에 1 frame 실패 후 같은 맵으로 relocalization 성공.
- features2400: 기존과 동일한 45 frame 실패 구간. 모든 입력 처리 후 `Shutdown` 로그 뒤 SIGSEGV. Atlas·최종 keyframe CSV·runner_complete가 없다.
- features2400_repeat_gdb: 159.417277832초 1 frame 실패 후 복구; 기존과 같은 45 frame 실패 구간 뒤 새 맵. 정상 종료·Atlas 저장·최종 CSV를 확인했다.

## 확정된 기전과 남은 가설

소스 줄 번호와 상세 설명은 [source_mechanism.md](source_mechanism.md)에 있다.

- **확정:** runner는 non-inertial `STEREO`로 좌우 PNG만 입력한다. 별도 depth 공백이 ORB에 직접 전달되지는 않는다 (`runner.cpp:28,53–60`).
- **확정:** timestamp 역행은 새 맵을 만들지만, `>1초` jump에 의한 reset 처리는 inertial 모드 안에 있다 (`Tracking.cc:1820–1853`). 이 기록은 timestamp가 증가하며 최대 stereo gap도 1초 미만이다.
- **확정:** 특징 검출 수와 map matching inlier 수는 다르다. motion model이 실패하면 reference keyframe으로 fallback하고, 이후 relocalization·local-map inlier 기준을 만족해야 한다 (`Tracking.cc:1953–1955,2730–2735,3030–3060,3757`). 따라서 1000개 이상의 ORB 검출만으로 추적 성공을 보장할 수 없다.
- **확정:** stereo는 첫 실패 이후 원본 timestamp로 3초를 넘도록 relocalization에 실패하면 새 맵을 시작한다 (`Tracking.cc:2003–2010,2271–2286`). 관측된 3.064초 구간과 일치한다. 상태가 같은 호출 안에서 0으로 바뀌므로 `LOST=4` row가 없을 수 있다.
- **확정:** 이전 valid map은 Atlas에 유지되며 전체 Atlas를 저장한다 (`Atlas.cc:58–76,316–332`, `System.cc:1408–1438`). 서로 다른 맵의 좌표를 같은 원점으로 단순 합쳐서는 안 된다.
- **가설:** 공백 구간의 실제 카메라 변위·시야 변화·blur가 3D correspondence를 줄였을 수 있다. motion model은 이전 상대 pose를 그대로 사용하며 이 경로에 Δt scaling이 없다 (`Tracking.cc:2211,2870`). 내부 match/inlier 계측이 없어 정확한 탈락 기준은 미확정이다.
- **가설:** async LocalMapping/LoopClosing과 mapper idle·queue·BA interruption이 재생 변동에 기여할 수 있다 (`System.cc:202,219`, `Tracking.cc:3103,3158,3187–3203`). thread schedule을 계측하지 않았으므로 유일 원인으로 단정하지 않는다. runner는 recorded 간격만큼 대기하지 않고 최대 속도로 재생하므로 설정 변경의 순수 feature 효과와 scheduling 효과를 단회 비교로 분리할 수 없다.
- **미확정:** 2400 첫 실행의 SIGSEGV 원인. 종료 신호와 `Shutdown` 뒤 발생한 사실만 확인된다. core는 비활성(`ulimit -c=0`) 상태로 생성하지 않았고 기존 crash 파일도 발견되지 않았다. 추가 GDB 1회에서는 정상 종료해 stack trace를 얻지 못했다. 특징 수 증가가 crash 원인이라고 확정할 수 없다.

## 판단과 다음 검증

현재 baseline을 유지한다. 특징 수 2400은 주 맵 분리를 없애지 못했고 tracking 중앙값이 약 40% 증가했으며 한 실행에서 저장 실패가 있었다. 정확도 또는 안정성이 개선됐다는 근거가 없다.

먼저 수집·전송·기록 경계의 timestamp와 drop reason을 계측해 입력 공백을 줄이고, 영상 품질·stereo sync·실제 match/inlier 수를 함께 기록하는 것이 타당하다. `RECENTLY_LOST` 구간의 pose를 정상 지도 pose로 소비하지 않고 새 맵의 좌표계를 결과에 표시해야 한다. 임계치 연장이나 motion model 변경은 별도 계측·회귀 검증 대상으로 남긴다.

외부 GT pose와 시간 정렬로 ATE/RPE·metric scale·loop consistency·실패 복구를 검증해야 한다. 갈라진 맵 전체를 하나의 rigid alignment로 평가하거나 비정상 frame 감소를 GT 정확도로 해석해서는 안 된다.

## 보존한 증거

- [comparison.json](comparison.json): 실행별 상태·구간·최종 저장 여부·처리시간.
- [comparison.csv (별도 자료)](../../../data-assets.md): 원본 timestamp별 5개 실행의 state/map ID와 입력 gap.
- [verification.json](verification.json): 3604개 원본 timestamp 일치, input/settings/binary/runtime/vocabulary hash 일치, complete 실행의 최종 CSV 수 검증.
- 각 실행 폴더의 manifest.json, runner.log, timings.csv, summary.json. complete 실행은 Atlas와 최종 map keyframe CSV도 보존.
- [environment_provenance.json (별도 자료)](../../../data-assets.md): 실행환경과 native/vocabulary/input SHA-256.
- [replay_probe.py (별도 자료)](../../../data-assets.md): 같은 입력을 새 하위폴더에서 실행하는 재생 스크립트. `label`은 기존 폴더와 겹치면 실패하며 덮어쓰지 않는다.
