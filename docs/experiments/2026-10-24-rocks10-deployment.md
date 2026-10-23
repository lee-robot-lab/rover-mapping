# rocks10 배포 연결

요구: Claude가 만든 장애물 맵으로 MPPI를 계산하고 기존 RUN1~4·앵커를 유지한다. 사이트 배경에는 돌을 미리 표시하지 않으며 현재 위치 반경35cm 안에서만 나타낸다.

- 선택: `mission_fillet25_rocks10`, 지름10cm 돌4개, 기존 격자 대비 차단56셀 추가.
- 배포: mission site/grid 교체, mission identity만 변경, 시나리오·구역·RUN 정책 유지.
- 표시: 본 주행 캐시 재생성, TELEOP MPPI 동일 site, 주변 코스트 투영. 바구니 전이 영역과 배경 PNG 보존.
- 일관성: 실기 status→relay→UDP identity, 시작 시 terrain/cache 대조, 수신 맵 불일치 시 예측 숨김, provenance에 cache 원본 hash 기록.
- 검증: map 단위 검사, 네 방향×네 RUN 기구학 UDP→HTTP, Pi 관련 CLI와 중계 회귀 검사. 실기 모터·센서·물리 접촉 검증은 별도다.

대형 원본 STL/mesh는 배포하지 않는다. 실기와 중계는 `load_mesh=False`로 검증된 grid를 읽는다. 배포가 Isaac 장면 또는 실제 장소의 장애물을 생성하지는 않는다.

돌 표시는 `launch.py --hide-stones`로 끌 수 있다. 이 옵션은 중계 서버에만 전달하며 제어·맵·코스트·슬립·MPPI 예측은 바꾸지 않는다. 기본은 표시 ON이다.

실행 결과: 회귀 검사183개 통과, 프런트엔드 Canvas 검사 통과, CLI bash30블록 구문 통과, 로컬 ksrc check 통과. 네 방향×네 RUN 16/16 complete, AUTO 표시 일치율90.2~100%, HTTP p95 58.9ms(1483회). TELEOP UDP→HTTP 30회 통과(p95 15.3ms). 캐시55,024상태/42,850,404바이트. 전체 원본 fingerprint가 cache coverage와 일치함을 확인했다. 실제 Pi 배포·실주행은 이번 검증 범위 밖이다.
