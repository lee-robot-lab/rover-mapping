# Tracking continuity and recovery implementation plan

> **For agentic workers:** Use subagent-driven-development for the isolated native and live-input tasks; parent owns replay/integration and release.

**Goal:** 실시간 프레임 생략과 짧은 재위치 추정 대기로 발생하는 지도 분리를 줄이고 기존 지도 복귀를 지원한다.

**Architecture:** 저장된 프레임 경로를 bounded FIFO로 추적기에 전달하고, 관측 시각화는 최신 결과 위주로 갱신한다. native stereo의 복구 대기는 설정 가능한 15초로 늘린다. 추적이 유효하지 않은 동안 새 위치나 관측을 만들지 않는다.

**Tech Stack:** Python3.10/PyQt5/OpenCV, official ORB-SLAM3 pinned native C++14, 기존 D435/Pi capture와 보정 IMU 동시기록.

**Spec:** 사용자가 승인한 세 항목 중 입력 연속성·기존 지도 복구 대기 개선을 이번에 구현한다. IMU 관성 융합은 별도 acquisition clock/축 검증 후 진행한다.

## Constraints and checks

- 원본 raw·기존 replay 결과·old native runtime을 보존하고 비교 결과는 새 폴더에 쓴다.
- bounded path metadata만 큐에 보관하며 이미지 원본 기록을 block하지 않는다. 초기 vocabulary 로딩 중에도 stereo pair를 보존한다.
- recovery 중 유효 pose가 없으면 coverage/지도에 추가하지 않는다. 서로 다른 map을 좌표 정합 없이 합치지 않는다.
- 기존 non-stereo/IMU의 recovery 정책을 바꾸지 않는다. 기본 설정이 없는 stock 동작은 유지한다.
- native patch의 4초 이상 영상 손실→동일 장면 복귀를 실제 runner로 A/B 검증한다.
- 최근 실제 기록의 실시간 형태 replay와 매핑 Python 검사, native shutdown/설치 및 앱 재시작을 확인한다.

## Tasks

- [x] 1. 연속 stereo queue·depth 대기와 시각화 부하를 최소 변경하고 실패 테스트→통과를 확인한다.
- [x] 2. stereo recovery timeout patch를 작성·재빌드하고 baseline을 보존한다.
- [x] 3. blank-gap 복귀와 저장 기록 replay를 실행하고 개선·한계를 기록한다.
- [x] 4. 전체 매핑 검사 후 runtime 교체·앱 상태·공유본 검증을 확인한다.

진행: 원본219통과, 공유본219통과/1skip, runtime 교체와 capability 로딩 확인. Qt stale fallback은 offscreen 실제 widget으로 검사했다. Pi hostname 조회 실패로 새 실기·자동 촬영 재시작은 하지 않으며 다음 앱 실행부터 새 runtime을 사용한다. GitHub 배포 결과는 이 파일을 포함한 커밋의 원격 HEAD로 확인한다.
