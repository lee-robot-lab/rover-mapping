# Execution ledger — 2026-10-13-handheld-mapping-app

- 사용자 요청: 계획부터 완성까지 추가 응답을 기다리지 않고 직접 수행.
- 환경: native `/usr/bin/python3` 3.10.12, conda unset. system numpy1.21.5/OpenCV4.5.4/PyQt5. Open3D는 독립 `.venv-mapping`.
- Ruling: 이 폴더는 Git 저장소가 아니므로 Git 초기화/commit/worktree를 만들지 않는다. 기존 파일은 그대로 두고 신규 모듈로 격리한다.
- Ruling: 사용자에게 추가 검토를 기다리지 말라는 직접 지시가 있으므로 스킬의 중간 승인 단계 대신 설계/계획을 저장한 후 실행한다.
- Ruling: 하드웨어가 연결되지 않은 동안 Pi접속·실기성능 주장을 하지 않는다. 실제 Open3D와 로컬 subprocess/Qt로 검증한다.
- Pre-flight: storage index의 ns timestamp를 reconstruct와 ORB 비교가 공통 사용. 최종 keyframe pose는 map별 camera-to-world. demo는 알고리즘 정확성 평가가 아니라 전체 동작/융합 기하 검증.

- Task 1: complete — protocol/storage/실제 Open3D TSDF, 원본 단위·회전·map 분리·잘린 전송 검사.
- Task 2: complete (실기 미검증) — Pi worker와 SSH controller 구현. local subprocess 정상/비정상 종료·disk failure 소유권 검사 통과. 실기 접속은 하지 않음.
- Task 3: complete — Qt 시작/종료/닫기 검증과 합성 전체 실행, app_preview.png/TSDF mesh 생성.
- Task 4: complete — upstream 핵심 C++ build/runner와 같은 입력의 ORB 기준 비교. C header/OpenCV include, EDLines source, Pangolin runtime, Eigen native 정렬 불일치 문제를 수정. 동일 Isaac Sim stereo180쌍 실제 재생 정상 종료: 점선44KF, 점39KF, 둘 다 추적100%/map0. 처리시간 점선p50 26.63ms/p95 31.73ms, 점p50 10.21ms/p95 11.73ms. O1/O3 차이·GT 없음·Pi 미검증을 명시.
- Task 5: complete — 새 app smoke42프레임 촬영→원본/최종 자세→TSDF 37,372vertices/73,638triangles, 완전 종료. app_preview.png를 실제 Qt 캡처로 갱신. 사용 안내/검증 README와 계획을 마감.
- Verification: 최종 신규25+기존69=94 passed in5.60s. pyflakes/compileall/bash-n 통과. full_pytest.log 전체 수집은 기존 trimesh/mujoco/torch 누락으로4 errors. 관련 없는 ML 환경은 변경하지 않음.
- Ruling: RealSense ROS Z16은 fix_depth_scale에서 mm로 변환됨. 센서 depth_units를 TSDF scale에 직접 사용하지 않고0.001m와 source provenance를 기록한다. 잘못 사용하면 지도 scale이 틀어지므로 공식 wrapper 소스를 확인함.
- Ruling: 노트북에서 같은 원본을 점 ORB와 점선으로 모두 재생하는 runner를 추가해 Pi/live와 laptop/offline 시간의 잘못된 직접 비교를 피한다. 실기 정확성이나 Pi 속도의 증명은 아님.
- Final review: author self-review; 별도 agent는 사용하지 않음. 프로세스 소유권·SIGHUP·timestamp/TF·파일 성공 조건·기존 실행 보존을 점검.
- 추가 회귀: 이전 실행의75초 watchdog이 새 실행을 죽이지 않도록 generation guard, 종료 후 elapsed/FPS 고정, 서로 다른 점선 pose/TSDF가 기존 점 결과를 보존하는지 검사.
- 마지막 correctness review: 일부 keyframe depth 누락을 완전 완료로 표시하던 문제와 점선 TSDF partial의 UI 전파 누락을 각각 RED로 재현→수정→전체94개 GREEN. 원본/기본 ORB 지도는 보존한다.
- 검증 증거: `outputs/handheld_mapping/20261013_app_validation/README.md`; Pi와 카메라 연결은 하지 않았으며 실제 장치 검증까지 완료했다고 주장하지 않음.
