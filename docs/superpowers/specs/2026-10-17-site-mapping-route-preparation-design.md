# 팀원 STL 입력 이후 대표점 방문·경로 전환 준비 설계

상태: 구현 전 검토용. 2026-10-17 사용자 범위 수정: 매핑·STL 생성은 팀원 담당이다.

## 목적과 범위

B는 사전 맵이 없는 여의도 중간 테스트 기회다. 이번 입력은 팀원이 제공할 STL이며, 매핑 앱 변경·STL 생성/export는 제외한다. 실제 대회 사천에서는 사천 prior와 사천 현장 관측 지도를 별도 보존·정합한다.

준비할 흐름은 **STL 불러오기 → 단위/up·출발/갈림길/정상 설정 → 세 경로·대표점 설정 → 저장·재로드 → 등판 시도·후퇴·우회·정상 방문·복귀 실행/재생**이다. 실제 STL 전에 합성 지도 demo로 검사한다. 코드 기준은 /home/su/rover-mapping, Python은 기존 mapping venv다. 새 ML/ROS 의존성은 설치하지 않는다.

## 현장 설정

STL 단위 m/mm를 명시하고 up 또는 rigid transform으로 중력 방향을 맞춘다. 실제 경사는 유지한다. 출발 position/heading, 갈림길, 공동 정상 대표점, 10°·15°·30° 길의 경유점과 진입부 대표점을 클릭·수정한다. 좌표를 코드에 고정하지 않는다. 경로 이름의 각도와 STL의 측정 경사는 구분한다. 상행·하행은 별도 directed route다.

30°는 명시적 시험 구간이다. 일반 경로의 slope 제한을 전역으로 풀거나 unknown/장애물을 통과 가능으로 바꾸지 않는다. geometry만으로 모래 견인력이나 실기 등판 성공을 확정하지 않는다.

## 임무 흐름

출발 → 갈림길 → 30° 진입부 방문·등판 시도 → 진행 부족이면 실제 지나온 구간 후퇴 → 15° 상행 → 정상 방문 → 10° 하행 → 출발지 복귀. 15°도 실패하면 10° 상행으로 전환한다. 30°가 성공하면 실제 성공을 기록하고 정상/복귀 단계로 진행한다. 실패를 미리 정답으로 넣지 않는다.

한 상행 실패는 공동 정상의 도달 불가가 아니다. 진입부 방문·등판 결과·정상 도착·복귀 완료를 구분한다. 모든 상행 실패 또는 복귀 경로 부재는 이유와 함께 실패로 남긴다. pose lost/stale·역행·미정렬 입력에서는 정지 요청/판정 보류이며 도착이나 등판 실패를 확정하지 않는다. 관측 진행 부족은 stuck이며 encoder·기구학·독립 이동 추정 없이 wheel slip이라 부르지 않는다.

방문 상태는 pending/active/reached/unreachable/failed/aborted다. 도착 tolerance, 시도 제한, 진행 문턱은 설정값이며 실측 검증값이라 주장하지 않는다. 후퇴·fallback은 실제 pose/결과 입력으로 결정하고 demo의 합성 입력과 분리한다.

## 구현 단위

1. site_map.py: STL hash·단위·rigid transform을 보존하고 bounded 2.5D grid 생성. 높이·경사·단차·unknown·벽·footprint 여유를 검사한다. 구멍을 보간해 free로 만들지 않고 다층/지면 불명확 영역은 unknown으로 둔다.
2. site_mission.py: 출발/갈림길/정상·대표점·세 directed route·정책 검증·저장·재로드. 기존 GridMap/A* 기초를 필요한 범위에서 재사용한다. 사용자 경유점 경로와 자동 연결 구간을 구분한다.
3. mission_editor.py: 별도 Qt 창의 3D/XY 지도·출발 heading·대표점·세 경로·Undo·검사·저장·재로드·demo. 기존 매핑 앱을 확장하지 않는다.
4. mission_runtime.py: GUI 없는 임무 상태 관리·경로 추종·진행 부족 감지·실제 실행 구간 후퇴·fallback·다른 경로 복귀. frame/timestamp/validity 검사와 bounded body velocity 요청 출력.
5. mission_replay.py: 합성 3경로의 성공·정체·lost·복귀 실패 재생, JSONL/CSV·경로 그림 생성.
6. tools/plan_site_mission.sh, tools/run_site_mission.sh: 기존 mapping venv 사용. 저장본과 timestamped JSONL pose 입력으로 상태/속도 요청을 기록한다. 기본 동작은 실제 모터 송신이 없으며 firmware/CAN 코드를 수정하지 않는다. 실기 adapter는 정의한 입출력 계약에 연결한다.

## 저장·실행 계약

새 폴더에 site.json, grid NPZ, mission.json, 방향별 route CSV, preview를 저장한다. 원본/이전 결과는 보존한다. site에는 STL SHA256·원본 단위·m 변환·frame·transform·grid origin/resolution·설정·생성 코드 버전을 기록한다. mission은 해당 지도 해시/frame에 묶고 다른 지도로 바뀌면 재검사를 요구한다.

Runtime 입력은 stamp_s, frame, position_m, yaw_rad, tracking_valid, 선택 encoder 속도와 receipt 시각이다. raw ORB를 검증 없이 site 좌표로 사용하거나 서로 다른 장치 clock을 동일시하지 않는다. 출력은 현재 구역/경로/방향·상태·bounded vx/yaw_rate 요청·유효시각·정지 이유·event다. GT는 demo 입력에만 명시한다.

실제 제어 송신/firmware 변경은 기본 동작에 포함하지 않는다. 실기 adapter 연결 전 시뮬레이션/fake 통신으로 정지·후퇴·전환을 검증한다.

## 완료 검증

비대칭 경사 STL의 m/mm·up·hash·bounds·grid origin과 벽/단차/구멍/unknown/footprint/slope를 검사한다. 빈/NaN mesh·잘못된 단위/transform·크기 상한·source hash 변경을 거부한다.

Qt offscreen에서 실제 클릭·경로 편집·Undo·저장·재로드·demo를 검사한다. replay는 30° 성공, 30° 실패→15° 성공→10° 하행, 15° 추가 실패→10° 상행, 모든 경로 실패, pose loss/stale/역행, 복귀 실패를 포함한다. 기존 mapping/headless 전체 회귀·tool syntax·CLI를 검증하고 native skip·실제 STL 미제공·실기 미검증을 구분한다.
