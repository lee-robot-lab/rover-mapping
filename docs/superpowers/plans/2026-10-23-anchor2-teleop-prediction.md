# 네 출발 방향과 TELEOP 보조 예측

사용자 승인 범위: 동일 출발좌표(3.60,2.82)에 ANCHOR1=-y, ANCHOR2=-x, ANCHOR3=+y, ANCHOR4=+x. 기존 op5/anchor 명령은1을 유지하고 op7/anchor2,op8/anchor3,op9/anchor4 추가. 정지3초·센서정상·TELEOP 제한 유지. 실제 로버가 그 방향을 보고 있을 때 운영자가 선택한다. 시뮬은 실제 가상 자세가 해당 방향8도 이내일 때만 ANCHOR를 적용하며 회전시키지 않는다.

TELEOP MPPI는 노트북 읽기 전용 모델이다. 현재 pose/v/w에서 짧은 국소 목표를 만들고 동일 지도/MPPI로 후보·선택궤적·슬립 근사를 표시한다. 미션 목표·모터/조이스틱 제어에는 반영하지 않는다. AUTO는 회차·단계별 offline 캐시 유지. 정지·stale·지도밖/계획불가 예측은 숨긴다. Pi 계산/센서 payload는 추가하지 않는다.

- [x] 실행기/gateway의 네 preset·START 검사·runtime 방향·회귀 검증.
- [x] GUI/CLI 십자 화살표 버튼/명령과 선택 preset 상태 표시.
- [x] 시뮬 동일 preset 연결과 −Y/−X 두 방향 캐시+네 방향×4회차 완주 검증.
- [x] TELEOP 모델과 server/UI 통합: 정지/stale/AUTO 분리·읽기전용 확인.
- [x] CLI/README, 전체 테스트, 출처/캐시 오차 검증, commit/push.

사용자 실행 중인 시뮬레이터/사이트는 종료하지 않는다. 검사 프로세스는 별도 네트워크 namespace에서 실행한다.

최종:168 pytest passed, 네방향×4회차16조합 complete(AUTO match90.2~100%), TELEOP 전진/횡이동30회 UDP→HTTP p9517.1ms·모델계산p9511.4ms. 소스hash·좌표압축·frontend·CLI shell/링크검증. GUI640×732 실렌더검토. AUTO초기회전캐시밖숨김은유지. 사용자시뮬레이터는종료하지않음.
