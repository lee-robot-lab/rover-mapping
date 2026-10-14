# SDD ledger — plan: docs/superpowers/plans/2026-10-13-live-mapping.md
Ruling: 사용자가 이전 전체 구현부터 직접 완료하도록 요청했고 현재 직전 화면 설계를 승인했다. 추가 검토 대기 없이 실행한다.
Ruling: Git이 없는 workspace이므로 Git 전용 sdd/worktree/commit 작업 대신 이 ledger와 검증 artifact를 유지한다.
Pre-flight: Task1 update/snapshot은 Task2 worker와 Task3 UI가 공유. pose는 camera-to-world, 투영은 initial optical X-Z로 통일. Task2 mailbox가 raw writer 경로만 받으며 raw 보존을 변경하지 않는다.
Pre-flight: IMU 기록/관성 보정은 별도 미완료 작업이다. 이번 UI를 IMU 융합 또는 중력 정렬이라고 표시하지 않는다.
Task1 complete: test_live initial4tests RED(ModuleNotFound)→GREEN; no-depth camera pose regression RED(AttributeError)→GREEN. Persistent coverage state2,240,000B; pose/path bounded.
Task2 complete: native '-' input RED(exit1,noCSV)→native build GREEN; real D435 archived replay produced64pose rows/1475cells before EOF, final65rows/1493cells, owned exit0. last received-to-pose20.6ms excludes network/display. 53live pairs dropped during initial vocabulary warmup; raw source remained untouched. Longer steady replay pending.
Task3 progress: demo/session/widget initial3tests RED→GREEN; live finish summary I/O failure RED(failed session)→GREEN(final TSDF complete with live_error). Existing+new142tests passed before final guards/review fixes. Qt demo screenshot inspected.
Final review: live_review read-only Important1 (parent exits leaving descendants) and Minor1 (pose lag includes depth wait). Important reproduced RED with SIGTERM-ignoring child; cleanup now signals owned group even after parent exits, escalates surviving group. Minor fixed by timestamping pose receipt before depth wait. Reviewer declined live Pi/GT/IMU validation: agree, retain explicit limitations.
Task3 complete: 실제 저장영상20초 replay에서 Qt+raw+native live+final ORB+TSDF 완료; UI frame inspected. UI status82samples lagmedian20.51ms/p9520.71ms,10초 이후 live 추가skip0. 현장 Pi/network 미검증. source label을 calibration metadata로 보존하는 regression RED(D435_Pi 오표기)→GREEN 수정.
Final review: 수정된 owned group cleanup/lag 정의를 reviewer가 재검토하고 신규14tests 재실행 후 승인 가능 판정. 새로운 Critical/Important 없음. 지도archive/runtimefailure/종료I/Oerror regression GREEN.
Ruling: IMU integration은 앞선 미완료 작업이며 이 feature가 활성화하지 않는다. Pi 연결과 실제 clock/extrinsic 보정 뒤 진행한다. 현재 live는 카메라 기준 참고 투영으로 표시한다.
Ruling: Git branch/merge/commit은 없는 workspace에서 수행하지 않는다. 산출물·테스트로그·이 ledger를 유지한다.
Final verification: handheld/headless146passed in10.53s; pyflakes/compileall/bash-n GREEN. Updated native Qt mapping window launched idle with live checkbox defaultON. Pi DNS lastcheck timeout; no real recording/motor commands started.
