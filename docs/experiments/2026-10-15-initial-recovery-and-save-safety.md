# 초기 지도 복구·Atlas 저장 안전성 — 2026-10-15

앞선15초 복구 대기 변경에서 남은 **KF0만 있는 초기 지도 복구 실패**를 보완했다. 별도 실제 native probe에서 재현한 **저장 관측 정리 중 SIGSEGV 두 경로**도 수정했다. ORB inlier/PnP/기하 임계값은 그대로이며 IMU는 여전히 동시 기록만 한다.

## 초기 지도 복구

upstream은 KF0을 LoopClosing queue에서 제외한다. keyframe database 등록은 이 queue의 처리 경로에 있어서 첫 키프레임만 있는 지도에는 재위치 추정 후보가 없었다. 앞선 버전은 지도를15초간 유지해도 같은 초기 장면에서 복구하지 못했다.

최신 `orbslam3_stereo_recovery_wait.patch`는 **`STEREO && wait > 3 && KF id == 0`**일 때 BoW 계산과 database 등록을 LocalMapping thread에 넘기기 전에 수행한다. KF0의 기존 queue 제외 정책과 겹치지 않으므로 중복 등록하지 않는다. 복구는 계속 원래 PnP/inlier 검증을 통과해야 한다. 설정 없는3초 기본값과 IMU 복구 정책은 바꾸지 않는다.

실제 D435 기록의 처음120쌍(키프레임1개), blank61쌍(합성 timestamp 약4.07초), 첫 키프레임 장면45쌍을 사용했다. 이전 core에서는 복귀45쌍 전부 복구 실패해 회귀 검사가 실패했다. 수정 후에는45쌍 모두 같은map0에서 추적했다. 낯선 random stereo는 복구로 인정하지 않았고, blank241쌍(약16.07초) 뒤에는 새 지도에서 시작했다. 모든 blank 구간은 유효 pose가 없었다.

## 저장 중 삭제와 순회

실제 `System/Map/MapPoint/KeyFrame`을 사용해 수집한 포인트의 관측 소속을 바꾸거나 저장 대상에서 제외했다. allocator perturb를 켠 같은 native core에서 다음 두 경우가 `-11(SIGSEGV)`로 종료했다.

1. `Map::PreSave`가 다른 지도에 속한 keyframe 관측을 제거하면 `EraseObservation → SetBadFlag`가 현재 point를 `mspMapPoints`에서 지운다. range loop의 iterator가 무효화된다. GDB에서 `std::_Rb_tree_increment → Map::PreSave` stack을 확인했다.
2. `MapPoint::PreSave`가 저장 대상에서 빠진 keyframe의 관측을 제거하며 현재 관측 iterator 또는 남은 관측 전체를 무효화한다.

새 `orbslam3_presave_observation_safety.patch`는 Map의 정리·backup 순회에서 callback 전에 iterator를 진행한다. point의 관측은 작은 복사본으로 순회하고 전체 관측이 비워지면 중단한다. 마지막 관측 제거 후 참조 keyframe은 `nullptr`로 만든다. 관측 정리 후 bad point가 된 항목은 backup에 넣지 않는다. 포인트 전체 목록이나 영상 원본을 복사하지 않으며 임시 메모리는 현재 point의 관측 수에 비례한다.

같은 probe에서 두 충돌이 사라지고 마지막 참조 제거도 정상 종료했다. 유효 관측은 저장 정리 전후 point 수와 참조를 유지하는 positive control로 검사한다. 과거 실기 종료 충돌 모두의 원인이 확정됐다는 의미는 아니다.

## 설치와 검증

- native 실제 통합 검사6개: **6 passed (51.73초)**. 초기 지도 known/unknown/timeout3개와 foreign observation/last observation/missing keyframe3개다. 촬영 데이터와 native 자산이 없으면 이6개는 skip한다.
- 공유본 전체 검사: **219 passed, 7 skipped (34.83초)**. 위6개 외에 Git에 포함하지 않는 선택 native 자산 검사1개가 skip이다. 실제 native 검사는 작성 머신의 원본 workspace에서 따로 수행했다.
- 최종 core의 저장 기록1742쌍 offline replay: 추적 성공1705/1742(**97.88%**), map0 유지, 정상 저장·종료. 중앙값15.31ms/p95 24.31ms는 해당 replay의 `TrackStereo` 시간이며 전체 지연·Pi FPS·정확도가 아니다. 비동기 스케줄과 실행 부하에 따라 반복 수치는 달라질 수 있다.
- stock source에 C++14/shutdown/Atlas tracking cache/최신 stereo recovery/PreSave safety **5개 patch를 순서대로 적용**했다. core와 runner를 재빌드·설치하고3개 capability marker를 실제 로딩했다. 구 core는 시작 전에 명확히 거절했다.
- 독립 review에서 남은 Critical/Important finding 없음. 새 Pi·USB·무선 실기와 GT 정확도·중력 정렬·IMU 관성 융합은 별도 검증이다.

새 PC와 이전 `51ece58` 설치의 갱신 방법은 [설치 설명서](../installation.md)를 따른다. 작성 머신의 runtime은 이미 교체했으며 다음 매핑 앱 실행부터 사용한다.

```bash
ROVER_STEREO_RECOVERY_TEST_RUN=/absolute/path/to/saved/RUN \
QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m pytest -q tests/handheld_mapping/test_native_initial_recovery.py
```

이 시험은 처음120쌍이 정지한 초기 지도인 D435 기록을 기준으로 한다. 임의의 촬영 기록 모두가 동일한 지도 성장 조건을 만족하지는 않는다. 원본 PNG/보정 JSON/native 바이너리는 Git에 포함하지 않는다.
