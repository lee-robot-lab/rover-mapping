# 노트북 실시간 ORB·촬영 범위 안내

사용자 승인: “그렇게 하자 실시간이 더 좋을것 같아”. 직전 제안인 카메라 옆 관측 지도·카메라 위치/시야·정지 프레임 중복 억제·추적 손실 표시·종료 후 최종 TSDF를 구현한다. 앞서 사용자가 계획부터 완성까지 직접 진행하도록 지정한 실행 방식을 유지한다.

## 목적과 범위
촬영하면서 아직 보지 않았거나 다른 시점에서 추가로 볼 구역을 판단한다. Pi는 현재 센서 전송을 유지하고 노트북에서 native ORB를 별도 실행한다. IMU·encoder 기록/보정/관성 융합은 앞선 별도 후속 작업이며 이번 실시간 기능이 적용했다고 주장하지 않는다. 모터·조종 프로세스는 건드리지 않는다.

## 입력과 계산
- 기존 원본 기록 후 그 파일 경로를 실시간 ORB에 전달한다. capture thread는 ORB를 기다리지 않는다.
- 정확히 같은 시각의 좌우 영상만 pairing한다. pairing 대기 최대8stamp, live 대기2쌍. 포화 시 오래된 live 쌍을 버리고 별도 누락을 집계한다. 원본 저장은 그대로 유지한다.
- native baseline runner의 기존 offline 인터페이스를 보존하고 stdin 쌍 입력과 flush한 pose CSV를 추가한다. camera-to-world pose, tracking state, map identity, 처리시간을 매 입력마다 출력한다. stdout 로그는 pose protocol로 쓰지 않는다.
- valid pose와 2ms 이내 depth만 융합한다. depth는 calibration의 scale/T_left_depth를 적용하며 .25–3m, stride8로 제한한다.
- 20m×20m/5cm 고정 격자. 초기 optical world의 X-Z 투영이며 중력 정렬이 아님을 명시한다. 범위 밖 점은 버리고 집계한다. 카메라 위치를 로버 중심으로 표시하지 않는다.
- 회색=미관측, 노랑=1–2회, 초록=3회 이상 다른 시점 관측 참고값. 셀별 마지막 관측 위치에서10cm 또는 방향10° 이상 변화가 있어야 추가 집계한다. 높이·통행 가능성·정확도·완전한 시점 독립성 지표가 아니다. 벽 등 실제 유효 depth 표면의 투영도 포함한다.
- map이 바뀌면 이전 격자를 노트북에 저장하고 새 격자로 시작한다. 서로 다른 map을 겹치지 않는다. loop closure로 이전 poses가 수정될 수 있으므로 실시간 관측 지도는 잠정값이며 최종 지도와 동일하지 않다.

## 상태와 종료
- 별도 thread/process가 실패하면 실시간 안내 오류를 표시하고 raw capture는 유지한다. 실패를 추적 성공이나 미관측 안전으로 표시하지 않는다.
- 처리 지연은 노트북에 원본이 저장된 후 ORB 결과를 받을 때까지의 시간이다. 센서부터 화면까지 end-to-end latency로 부르지 않는다. 마지막 pose가1초 이상 오래되면 stale 표시한다.
- live thread는 소유 native process만 닫는다. 정상 종료 시 남은 최대2쌍 처리 후 stdin EOF/Shutdown. timeout 시 owned process group만 종료한다.
- 종료 후 기존 전체 raw offline ORB와 최종 keyframe TSDF를 유지한다. live drop이 있어도 raw가 보존되며 최종 재계산과 분리한다.
- UI는 IR 옆 coverage·경로·카메라 방향과 짧은 시야 부채꼴, map ID·추적 상태·지연·live drop을 표시한다. native 실행기가 있으면 live 기본 ON. 합성 demo는 알려진 pose임을 명확히 표시한다.

## 검증
평면 depth 변환, 정지 중복, 관측 시점 변화, map 분리, 범위/메모리 상한, 비유효 depth, exact stereo pairing, depth tolerance, 느린/죽은 runner가 capture를 막지 않는지, 종료/자식 cleanup, 기존132개 회귀. 실제 저장된 D435 기록을 native stdin으로 재생해 EOF 전 pose 출력을 확인하고 Qt 화면을 확인한다. Pi 네트워크 연결 시 실기 전체 부하 수신률은 별도 측정한다. Git 저장소가 아니므로 Git 초기화/commit/worktree를 만들지 않는다.
