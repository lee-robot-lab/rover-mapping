# RUN1~4 본 주행 표시 캐시 구현 계획

사용자 요청: 기존 바구니 연결 구간에 이어 본 주행 MPPI·슬립 표시 추가.

설계: 배포 주행 상태기·MPPI를 점기구학으로 실행해 4회차를 생성하고, 각 계획 위치 주변 lateral/yaw 변형을 같은 MPPI로 별도 계산한다. 본 주행 캐시는 transfer 자산과 분리한다. 수신 회차·단계·pose/v/w 허용 범위에 맞는 경우만 표시한다. Pi 제어는 수정하지 않고 정지 슬립·stale·캐시 밖 예측은 숨긴다. 실측/물리 검증으로 표현하지 않는다.

- [x] 회차/단계 분리와 실기 본 주행 표시·stale·불일치 회귀 테스트 작성 및 RED 확인.
- [x] core Lookup 단계 필터, decode 단계 보존, server 캐시 선택 구현.
- [x] 재현 가능한 offline 생성기를 배포 MPPI와 연결하고 4회차 캐시·범위/완료/크기 검증 생성.
- [x] 본 주행 합성 UDP→HTTP 표시·조회 지연, 관련 테스트, frontend 검증.
- [x] CLI/README 범위 갱신 후 기존 브랜치에 commit/push.

Python: 기존 /home/su/rover/.venv-mapping (numpy/scipy/pygame), 생성 시 BLAS 한 스레드.

추가 사용자 요청: `launch.py --sim`으로 로컬 기구학 로버+실제 조이스틱+사이트 연결. SPI/SSH/ROS 미사용. RUN 출발점 초기화, 실제 Gateway 준비·E-STOP·회차 선택, ORB 정상 가정. `verify_missions.py`에서 headless UDP→HTTP 4회차 검증.

최종 검증: 본 주행 캐시32,075개(24MiB), 4회차 complete, UDP→HTTP 표시 일치99.3~100%, local HTTP p95 60.1ms(1,615조회). 중계 관련pytest29개, frontend JS, source fingerprint·float16 origin 오차 검증. 실제 localhost 시뮬레이터와 조이스틱 GUI를8765에서 실행해 ready/신선한pose 수신 확인.
