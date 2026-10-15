# Stereo 입력 연속성·기존 지도 복구 — 2026-10-15

촬영 중 추적 손실로 지도가 분리되는 문제에 대해 노트북 입력 큐와 stereo 재위치 추정 대기를 수정했다. ORB-SLAM3 non-inertial `STEREO`를 유지하며, 보정 IMU는 동시 기록만 한다. 시계·축 검증과 관성 융합은 이번 범위에 없다.

## 조사와 변경

최근 `20261015_000854_aa8850`과 `20261015_001142_c10be6`은 live에서 지도 분리가 있었으나, 저장한 전체 IR의 offline 재생은 각각1081/1742쌍 모두 추적에 성공하고 map0을 유지했다. 첫 기록에는 Pi sender write 2467.7ms와 live 입력 공백이 있었고, 두 번째 손실 시작에서는 입력 간격이 정상15FPS였다. 전송 공백만으로 모든 손실을 설명하지 않으며 native의 비동기 local mapping도 영향을 줄 수 있다.

- 기존 최신2쌍 큐를 **FIFO256쌍**으로 바꿨다. 큐에는 파일 경로와 시각만 보관한다. 포화 시 새 쌍을 거절하고 `dropped_pairs`에 기록하며 이미 대기 중인 순서를 보존한다. 좌우 미매칭 metadata는 각각256개, depth metadata는512개로 제한한다. raw 기록은 별도 경로로 계속한다.
- 추적 루프의 depth 대기 최대150ms를 제거했다. IR 추적은 depth 도착을 기다리지 않으며 depth가 없으면 해당 관측색 갱신만 생략한다. 원본 depth는 종료 후 재계산에 사용할 수 있다.
- 유효 pose의 경로는 모두 반영하고, depth coverage/overlay 갱신은 최대5Hz로 제한한다. backlog 중에도 주기적으로 갱신한다. `stale`은 결과 수신 뒤 경과뿐 아니라 입력 대기부터 결과까지의 `lag_ms`를 포함한다. 과거 pose를 현재 위치처럼 표시하지 않는다.
- `Tracking.StereoRelocalizationWaitSeconds: 15.0`을 live/offline 설정에 넣었다. pinned upstream core용 patch가 stereo 복구 대기를 설정 가능하게 하며 설정이 없으면3초다. IMU 모드의5초 정책은 유지한다. 설정값은 유한한1~60초만 허용한다.
- 설정을 활성화하면 작은 stereo 지도도 즉시 초기화하기 전에 복구를 시도한다. 복구 중 기존 coverage·경로를 남기고 유효하지 않은 pose/관측은 추가하지 않는다. 실제 지도 ID가 바뀌면 이전 overlay를 제거한다.
- runner/build wrapper는 native marker `orbslam3_has_stereo_recovery_wait`를 확인한다. 구버전 core가15초 설정을 조용히 무시하지 않게 한다. 설치 시 공유 라이브러리는 임시 파일에서 atomic rename으로 교체한다.

복구 대기는 **영상 timestamp 기준**이다. 15초 동안 특징점과 기하 검증에 성공하지 못하면 기존 정책대로 reset/별도 지도 생성이 가능하다. 서로 다른 map 좌표계를 임의로 붙이지 않는다.

## 실제 native 비교

같은 저장 D435 IR420쌍으로 지도를 만든 뒤, 검은 영상61쌍(약4.07초)을 넣고 이미 지도에 들어간 keyframe 장면과 주변 영상45쌍으로 돌아왔다. 손실·복귀 구간 timestamp는 시험용으로 합성했다. ORB inlier/기하 임계값은 바꾸지 않았다.

| core | 초기420쌍 | 복귀45쌍 | 복귀 구간 지도 |
|---|---|---|---|
| 수정 전 | 420 tracking OK | 45 tracking OK | 새map75 생성 후 자연 병합으로map0 복귀 |
| 15초 복구 후보 | 420 tracking OK | 45 tracking OK | 분리 없이map0, `Relocalized!!` 확인 |

앞서120쌍으로 만든 지도는 keyframe이1개뿐이어서 수정 버전에서도 복귀하지 못했다. upstream은 첫KF0을 LoopClosing queue에 넣지 않고 keyframe database 등록은 해당 경로에서 수행하므로 이 경우 재위치 후보가 없다. 대기 연장만으로 모든 초기 지도에서 복구하는 것은 아니다. 실패 결과도 원본 workspace에 보존했다.

blank 구간은 유효 pose 없이61/61쌍 `state3`이었고, 후보는 복귀 첫 프레임부터map0을 추적했다. 별도16.07초 blank 시험에서는15.07초에 reset하고 복귀 시 새 지도를 만들었다. 설정 키를 생략한 새 core도3.07초에 reset했다. timeout을 넘긴 손실을 가짜 연결로 숨기지 않으며 기본 동작도 유지한다.

새 runner와 구 core 조합은 시작 전 `Stereo recovery patch missing`으로 exit1을 반환했다. 새 core와 runner의 실제 빌드·설치, marker 로딩과 설치 파일 hash 일치를 확인했다. native 바이너리와 촬영 원본은 Git에 포함하지 않는다. 새 PC는 [설치 절차](../installation.md)의 네 번째 patch까지 적용해 빌드한다.

## 기록 재생과 한계

`20261015_001142_c10be6`의 원래 노트북 저장 간격과 stream 도착 순서로 live 경로를 재생했다. 첫 후보에서 **1742쌍 전부 처리, tracking OK100%, map0, queue 거절0, 최대 대기62쌍**, 정상 종료를 확인했다. native `TrackStereo` 시간 중앙값14.72ms, p95 19.51ms였다. 전체 센서→화면 지연이나 Pi 처리속도 측정이 아니다.

backlog 중5Hz 갱신과 stale 보완까지 포함한 **최종 코드**로 다시 재생한 결과는1742쌍 전부 처리, tracking OK1712/1742(**98.28%**), map0, queue 거절0, 최대 대기74쌍이었다. 손실29쌍(약1.93초)과1쌍(약0.067초) 뒤 각각 기존map0으로 복귀했다. 정상 종료했으며 시간 중앙값14.49ms, p95 17.52ms다. native 비동기 처리에 따라 반복 결과가 달라질 수 있으며 추적 손실을 완전히 없앴다고 주장하지 않는다.

최종 원본 매핑 검사 **219 passed**, 공유 체크아웃 전체 검사 **219 passed, 1 skipped**다. 공유본 skip은 Git에서 제외한 선택 native 자산 검사다. 실제 설치된 core는 별도 로딩·빌드·replay로 확인했다. backlog 중 주기 갱신과 입력 지연 stale 회귀를 포함하며 독립 review의 두 finding을 수정 후 재검토했다.

원본 촬영에는 이미 전송 누락이 있다. 이 replay의 queue 거절0은 원래 촬영의 누락0을 뜻하지 않는다. 새 Pi·USB·무선 실기, GT 기반 위치 정확도, 중력 정렬, IMU 관성 융합은 별도 검증이다. 이전 live에서 관측된 native shutdown SIGSEGV의 일반적 해결도 주장하지 않는다.

확인 시점에는 Pi hostname 조회가 되지 않아 새 실기 촬영은 하지 않았다. 노트북 runtime은 교체했고 다음 앱 실행부터 적용된다.

촬영 중 `기존 지도 복구 중`이면 멈춘 뒤 이미 찍은 장면을 천천히 다시 보여준다. 충분한 겹침·텍스처가 있어야 재위치 추정이 가능하며, 원본을 남겨 종료 후 전체 영상으로 다시 계산한다.
