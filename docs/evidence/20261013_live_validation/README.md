이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# 실시간 촬영 범위 검증 — 2026-10-13 시작, 2026-10-14 완료

실제 카메라를 현재 연결해 촬영한 시험이 아니다. 2026-10-13의 실제 D435 저장 IR/depth를 재생해 노트북 프로그램을 검증했다. Pi 네트워크는 마지막 확인 시 조회되지 않았다.

## 프로그램 흐름
원본 기록 → metadata 최신2쌍 큐 → native ORB stdin → 즉시 flush된 pose CSV → 2ms depth 매칭 → fixed X-Z 관측 격자 → Qt. 종료 후 전체 raw offline ORB → 최종 TSDF를 유지한다. native 기본 offline/ORB-LINE 인터페이스도 보존했다. 모터/STM/조이스틱 변경 없음, IMU 융합 없음.

## 증거
- `native_red/runner.log`: 변경 전 argv4 '-'가 input open 실패(exit1/no pose CSV).
- `build.log`: point native runner build 성공. upstream Eigen deprecation warning은 남아 있다.
- `recorded_live/validation.json`, `native_green.log`: 실제 저장영상8초 pacing에서 EOF 전에64pose/1475cells, 최종65pose/1493cells. 초기 warmup53쌍 skip, source 원본은 symlink로 읽기 참조.
- `ui_recorded_runs/20261014_001505_fdf531/`:20초 원래 간격 replay에서 left298/right300/depth293을 새 검증 폴더에 저장. 원본에 있던 stream별 누락까지 억지로 채우지 않았다. replay sender 자체 drop0.
- `ui_replay_validation.json`: live245쌍, 초기 skip53,10초 시점skip53/종료skip53, live error없음. UI상태82sample lag median20.514ms/p9520.712ms. sensor/network/display를 포함한 latency가 아니다.
- 같은 replay의 offline298쌍 tracking100%, 최종98/98KF TSDF융합,415123vertices/568524triangles. GT 정확도/중력 정렬 검증 전.
- `ui_recorded_during_capture.png`: 실제 기록 재생 중 IR과 잠정 관측 지도 동시 표시를 직접 확인한 화면.
- `ui_recorded_after_save.png`, `ui_demo.png`: 최종 저장과 합성 knownpose 흐름. 합성을 실제 native 정확도 검증으로 사용하지 않는다.
- initial replay session.json source가 기존 controller 기본 D435_Pi로 표기되는 문제를 검증 중 발견했다. 실제 source는 calibration/이 README/ui_replay_validation.json에 명시한 recorded_real_D435_replay다. 이후 controller가 calibration의 source를 채택하도록 회귀 RED→GREEN 수정했다. 과거 검증 기록을 실기 촬영으로 주장하지 않는다.

## 회귀/리뷰
`pytest_final.log`: handheld + headless suite. 새로운 관측/실시간/session 테스트14개를 포함한다. 최종 실행 결과는 해당 로그 확인. compileall/pyflakes/bash-n 통과. 전체 repo ML/기타 실험 suite 결과로 확대하지 않는다.

fresh read-only reviewer가 parent 선종료 시 descendant가 남는 Important1개와 depth 대기가 pose lag에 포함되는 Minor1개를 재현했다. SIGTERM 무시 descendant 회귀 RED→GREEN, owned process group cleanup, depth wait 전 pose receipt 시각 기록으로 수정. reviewer가 새14개 tests 재실행 후 승인 가능 판정. runtime live실패·finish summary I/O 실패에도 원본/최종TSDF 보존, map전환 archive도 테스트했다.

## 제한
20m×20m/5cm initial optical X-Z 참고 투영(중력 미정렬). 회색 unknown/노랑1–2시점/초록3이상은 촬영 안내이며 안전/정확도 costmap이 아니다. 초기 native vocabulary warmup은 live skip을 만들 수 있으므로 추적 정상 후 이동한다. raw source는 보존한다. loop closure 후 기존 coverage를 재융합하지 않아 최종맵과 차이가 있을 수 있다. Pi 전송·현재 조종·카메라를 함께 켠 실기 시험은 별도로 해야 한다.
