# D435 handheld mapping app

사용자 승인: 2026-10-13, “계획 만들고 완성까지 진행해줘 나 이제 잘거라서”. 설계·계획 검토 응답을 기다리지 않고 이 세션에서 직접 실행한다.

## 목적
노트북의 일반 프로그램 창에서 시작과 종료 버튼으로 D435 handheld 촬영을 제어한다. 좌우 IR·depth·시각·보정·추정 결과를 노트북에 저장하고, 종료 후 최종 keyframe 위치를 이용한 TSDF mesh와 point cloud를 생성한다. 기존 ORB를 유지하고 ORB-LINE 비교 실행을 추가한다.

## 구성과 계약
- 노트북 Ubuntu22.04 Python3.10/PyQt5. ROS 작업은 native, conda를 섞지 않는다. Open3D는 프로젝트 전용 `.venv-mapping`에 설치한다.
- Pi Ubuntu24.04 native Jazzy Python3.12. 연결이 없는 현재 Pi SSH 접속은 시도하지 않는다. 실기 버튼이 눌릴 때만 SSH 배포·실행한다.
- SSH stdout에는 길이 헤더가 있는 binary protocol만 보낸다. 로그는 stderr/RAM. 원본 프레임은 제한된 queue로 전달하고 drop을 집계한다. Pi 새 데이터는 `/dev/shm`에만 보관한다. 연결 단절 시 관련 실행만 종료하고 불완전 상태를 기록한다.
- 기본 카메라 입력은 좌우 IR640x480/Y8/depthZ16/15Hz, projector OFF. 실제 CameraInfo/profile을 검사한다. depth 단위는 센서 조회 결과를 사용한다. depth→left optical 변환은 TF에서 얻는다. 보정값을 과거 시험에서 복사하지 않는다.
- ROS 프로세스는 새 세션의 것만 소유한다. 기존 카메라·ORB·로버 제어를 종료하지 않는다. 기존 publisher가 있으면 충돌을 알리고 시작을 거부한다.
- 기록한 raw 영상은 timestamp와 함께 저장한다. TSDF는 최종 map별 keyframe CSV와 timestamp가 맞는 depth만 사용한다. pose 손실 구간 보간·서로 다른 map 자동 병합 금지. Open3D 입력은 world→depth camera extrinsic, voxel 기본1cm, 측정 정확도 보장 아님.
- 종료 후 최적화 keyframe이 없는 기록을 성공 지도라고 표시하지 않는다. mesh가 비어 있거나 전송/저장/worker 종료 실패면 `failed` 또는 `incomplete`다. 원본은 보존하고 재생성을 제공한다.
- UI: 연결 대상, ORB 선택, 시작, 종료 및 지도 생성, 폴더 열기, 기존 기록 재생성, 상태/시간/수신률/drop/저장 위치/카메라 미리보기. 시작 중복 방지, 저장 중 재시작 방지, 창 닫기 시 실행 소유권 유지 후 정상 마감.
- ORB-LINE은 별도 upstream clone과 offline stereo runner로 준비한다. 동일 좌우 IR에서 두 backend의 추적 성공률/처리시간/map 분리 수를 리포트한다. ROS2/Pi 성능은 실제 검증 전 검증 완료로 표기하지 않는다.

## 검증
binary protocol 잘림/크기 제한, timestamp matching, 좌표변환 방향, 서로 다른 map 분리, 저장 실패, 중복 시작, SSH/worker 비정상 종료, 창 닫기. 실제 Open3D 합성 깊이 평면/단차로 기하 오차 검사. Qt offscreen에서 버튼 상태와 전체 모의 촬영→저장→mesh 경로 검사. 기존 headless mapping 회귀 검사. 카메라·Pi 실기 검증은 연결 불가로 별도 남긴다.

## 범위
모터·IMU·Nav2·AI HAT·height map은 변경하지 않는다. 초기 TSDF는 검증된 최종 keyframe만 융합해 정확성과 재현성을 우선하며, 모든 중간 프레임의 최종 pose 재계산은 포함하지 않는다. 현재 디렉터리는 Git 저장소가 아니므로 임의 git init/commit을 하지 않는다.
