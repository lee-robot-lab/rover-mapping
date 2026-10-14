이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# 2026-10-13 노트북 매핑 앱 검증

실행: `bash /home/su/rover/tools/handheld_mapping.sh`. 카메라 없는 시험은 `--demo`를 추가한다.
사용 안내: [handheld-mapping-app.md](../../experiments/handheld-mapping-app.md).

## 확인한 동작

- native Qt 시작/종료/닫기와 실제 local subprocess의 원본 저장→최종 자세 수신→Open3D TSDF 생성.
- 최신 전체 흐름: `final_demo/20261013_031916_bc4934/session.json`, 좌우 IR 각42장·depth42장, 전송 누락0, 42프레임 융합, mesh37,372vertices/73,638triangles. 입력과 자세는 합성이다.
- `app_preview.png`는 위 실행의 실제 Qt 창 캡처, `tsdf_preview.png`는 별도 합성 실행의 2m/1.7m 두 표면이다. 둘 다 실기 정확성 증거가 아니다.
- `mapping_tests.log`: 신규25개+기존 headless69개, 총94 passed (5.60s). 잘린 전송·단위/TF·회전·timestamp·분리 지도·점선 TSDF의 독립 자세/출력·일부 depth 누락 상태·점선 일부 누락 시 기본 지도 보존·빈 depth·디스크 실패·프로세스 소유권·이전 타이머·Qt 마감 검사를 포함한다.
- pyflakes, Python compileall, shell `bash -n` 통과.
- 마지막 review에서 일부 keyframe의 depth가 없을 때 완전 완료로 표시하던 문제를 재현하고 수정했다. 점선 결과의 일부 누락도 최종 UI 상태에 반영한다. 실패 재현은 `partial_keyframe_red.log`, `partial_line_red.log`, 수정 후 전체 회귀는 `mapping_tests.log`에 있다.
- ROS Humble native 환경에서 Pi worker 모듈과 사용하는 ROS 메시지/서비스/tf2 import 통과. Pi Jazzy에서의 실제 실행은 아직 확인하지 않았다.

## 점·선 비교

`stereo_sim/`은 기존 Isaac Sim 캡처에서 복사한 동일 stereo180쌍이다. 이번 비교에는 GT를 사용하지 않았으며 depth도 없다. 실제 노트북에서 C++ SLAM을 돌린 시험이지만 실제 카메라 시험은 아니다.

초기 ORB-LINE 실행의 pose 최적화 충돌은 g2o와 SLAM core 사이 `-march=native` 불일치로 인한 Eigen 정렬 문제로 조사했다. 관련 진단은 `orbline_gdb.log`와 `orbline_alignment_gdb.log`에 보존했다. 모든 line target과 runner의 옵션을 통일한 뒤 실제 재생에서 정상 종료·최종 keyframe 저장을 확인했다. `orbline_build.log`와 `stereo_compare.log`에 실행 기록이 있다.

최종 결과: `stereo_sim/orbline/comparison.json`, 각 runner의 `runner_complete.json`.

| 동일180프레임 | 추적 정상 비율 | tracking 중앙값 | tracking p95 | 최종 keyframe |
|---|---:|---:|---:|---:|
| ORB-SLAM3 점 | 100% | 10.21ms | 11.73ms | 39 |
| ORB-LINE 점+선 | 100% | 26.63ms | 31.73ms | 44 |

둘 다 map0 하나를 유지했다. 초기 실패 자료는 `orbline_loader_failure/`, `orbline_alignment_failure/`로 분리해 보존했다. 이 stereo 자료에는 depth가 없어 C++ 점선 pose를 실제 depth에 융합한 시험은 아직 하지 않았다. 별도 기하 테스트는 점선 pose를 선택하며 기존 점 TSDF를 보존하는지 실제 Open3D로 확인한다.

점 core는 기존 O3/native, line core는 빌드 메모리를 줄인 O1/native이므로 처리시간 차이는 알고리즘만의 차이가 아니다. 처리시간은 노트북 offline tracking 호출 시간이며 Pi FPS·capture-to-display latency가 아니다. 실제 개선 판단에는 동일 컴파일 옵션과 GT·실기 코스 평가가 더 필요하다.

## 확인하지 못한 항목

- Pi/D435가 연결되지 않아 이번 작업에서 SSH 배포·카메라 depth 추가 profile·Wi-Fi 대역폭·실기 TSDF 정확도는 실행하지 않았다.
- 기본 Pi 알고리즘은 기존 stereo ORB-SLAM3다. 점·선 모드는 촬영 종료 후 노트북에서 비교한다. Pi 점·선15FPS를 확인한 것이 아니다.
- TSDF는 최종 keyframe과 timestamp가 맞는 depth만 융합한다. 전체 영상의 dense pose 보간·map 자동 병합·중력 정렬·RGB 색상·heightmap은 구현 범위에 포함하지 않았다.
- USB2에서 IR2장+depth15FPS의 부하와 queue drop은 연결 후 반드시 확인해야 한다. UI의 수신 FPS는 노트북 저장 도착률이다.
- `full_pytest.log`: 저장소 전체 pytest는 기존 테스트 수집 중 trimesh/mujoco/torch 누락으로4 errors. 관련 suite는 `mapping_tests.log`를 기준으로 확인하며 기존 ML 환경은 변경하지 않았다.

새 코드/로그/Atlas는 Pi `/dev/shm`에 생성하고 원본은 노트북에 저장한다. 시작 전 Pi 기존 카메라/ORB 실행을 정상 종료하고, 종료 및 지도 생성 완료 뒤 전원을 끈다. 앱은 기존 실행을 자동으로 종료하지 않는다.
