# Pi 최종 f판 실행기 구현 계획

**목표:** 확정된 YW16→YW17/f판의 ESKF·RegionMission·MPPI를 Pi 실시간 센서와 gateway에 연결하고, 인터넷 없는 AP 내부망에서 Git에 포함된 CLI로 운용한다.

**합의:** depth OFF, run_policy 단독 SPI owner, 노트북 단절 시 AUTO 유지, ORB 자동 재시작·마지막 신뢰 pose에서 좌표 재연결·기존 임무 재개. 코어 배치와 plan 분리는 선택 옵션으로 둔다.

**구조:** `deployment/ksrc/`에 실행기·ROS 입력·policy telemetry wrapper·CLI와 f판 runtime/mission을 함께 보관한다. 기존 소스를 덮어쓰지 않고 f판 snapshot의 필요한 import만 vendoring한다. 실제 센서 설정은 현재 ORB YAML과 IMU calibration에서 만든다. 시뮬 calibration을 실기에 대입하지 않는다.

1. [x] f판 runtime·mission import closure를 Git에 포함하고 provenance hash를 기록한다.
2. [x] 실패 테스트 후 실행기 구현: 준비/START/STOP, run1/run2, 센서 freshness, 명령·VO 패킷, ORB 복구의 좌표 연속성.
3. [x] ROS 입력·정책 wrapper·CLI 구현: fake 기본, 별도 명시 real, 소유 프로세스만 관리, AP SSH 명령.
4. [x] 호스트 회귀·기록 재생·Pi fake CLI 실행·배포 SHA 검증. 실제 모터 실행은 제외한다.

**검토 기록:** 독립 리뷰에서 IMU 공백 뒤 복구 진입·정지 시간 창, 복구 중 재공백 안정화 초기화 문제를 확인하고 실패 회귀 테스트 후 수정했다. 추가 종료 시험으로 fork worker의 상속된 SIGTERM handler 문제를 재현하고 worker handler 초기화·유한 종료를 적용했다. 새 navigation 테스트 15개 통과. f판 재생은 pose 13,504개에서 최대 차이 1.49e-7 m, 단계 16개·코너 이벤트 6개 일치. CLI와 시험 절차서는 별도 문서로 제공한다.

**검증:** 새 runtime가 원본 f판과 일치하는지 확인한다. 센서 공백·map 변경·중복 timestamp·워커 고장·프로세스 종료에서 0 명령, 복구 후 pose 연속성과 임무 단계 유지, STOP 이후 재개 금지를 시험한다. 현장 순서를 README에 기록한다.

**주의:** 자동 복구의 마지막 pose 연결은 전역 재위치 측정이 아니다. 정지까지 이동한 거리·미끄럼·외부 이동을 정확히 복원하지 못한다. 기록에 recovery origin과 공백 길이를 남긴다. 실제 구동 전 시뮬 및 바퀴를 띄운 검증을 권고한다.
