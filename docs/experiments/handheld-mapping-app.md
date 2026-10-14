# D435 노트북 매핑 제어창

최신 통합 기준: [D435·Pi·로버 매핑 인수인계](../handoff/d435-rover-mapping-handoff.md) (2026-10-14). 아래 초기 검증/미검증 표현은 작성 당시 상태다. 이후 실제 촬영·TSDF·live 화면 검증과 입력 공백 계측까지 진행했으므로 현재 완료 범위는 인수인계서를 따른다.

후속 사전 구현: **Pi 센서 전송만 / 종료 후 노트북 ORB** 옵션과 별도 지형 정보·costmap CLI가 추가됐다. 노트북 native ORB 실행기가 있으면 센서 전송 옵션이 기본 선택된다. 기존 Pi ORB 모드도 유지한다. 아래 기존 흐름과 함께 [새 모드·메모리·지형 출력 안내](laptop-terrain-preparation.md)를 참고한다. 새 Pi/D435 모드는 실기 연결 미검증이다.

## 실행

```bash
cd /path/to/rover-mapping
bash tools/handheld_mapping.sh
```

카메라 없이 버튼·저장·TSDF 전체 흐름 확인:

```bash
bash tools/handheld_mapping.sh --demo
```

이 프로그램은 일반 Qt 창이다. 브라우저나 Foxglove 계정이 필요하지 않다. 초기 연결 대상은 `pi@raspberrypi.local`, 저장 루트는 `/path/to/rover-mapping/outputs/handheld_mapping/sessions`다. IP가 바뀌었으면 창에서 연결 대상을 수정한다. 프로그램을 여는 것만으로 Pi에 접속하지 않는다.

1. Pi에 D435를 연결하고 이전 카메라/ORB 실행이 있으면 사용자가 정상 종료한다. 앱은 기존 실행을 자동으로 죽이지 않는다.
2. **매핑 시작**: 새 날짜별 폴더를 만들고 SSH로 worker를 Pi RAM에 배포한다. 카메라/보정/ORB 준비가 끝나면 촬영 중으로 바뀐다.
3. 천천히 겹치는 시야를 유지하며 이동하고 시작 위치를 다시 관측한다. 카메라 영상과 추적 상태를 확인한다.
4. **종료 및 지도 생성**: 카메라·ORB 정상 종료, 최종 파일 전송, TSDF 융합이 이어진다. 저장 완료 전에 Pi 전원을 끄지 않는다.
5. **저장 폴더 열기**에서 원본과 `dense/` 결과를 확인한다. **기록에서 다시 생성**은 원본을 보존하고 TSDF 산출물만 재생성한다.

창 닫기/SIGINT/SIGTERM도 촬영을 정상 종료하고 마감이 끝나면 닫는다. 지도 생성/점선 비교 단계에서는 작업 종료까지 기다린다. 실패하면 폴더를 보존하고 오류를 표시한다. 전송 queue 누락 또는 생성하지 못한 지도 조각이 있으면 일부 누락 상태를 표시한다.

## 저장 구조

```text
sessions/YYYYMMDD_HHMMSS_ID/
  session.json                  실행 상태·수신 수·오류
  capture_calibration.json      이번 CameraInfo·TF·depth 단위
  frames.jsonl                  원본 timestamp(ns)·encoding·파일 경로
  events.jsonl                  상태·발행 pose·전송 metadata
  raw/left/*.png                좌측 IR 원본, mono8
  raw/right/*.png               우측 IR 원본, mono8
  raw/depth/*.png               depth 원본, uint16; RGB 미기록
  orb/                         기존 ORB Atlas·최종 keyframe·로그 등
  dense/map_ID_mesh.ply         TSDF 표면
  dense/map_ID_cloud.ply        TSDF 표면 점군
  dense/summary.json            융합 수·누락 수·좌표계·설정
  worker.log                   SSH/worker 오류
  orbline/                     선택 시 점선 실행·최종 keyframe·비교 결과
  orb_offline/                 설치 시 동일 노트북 입력의 ORB 점 기준
  dense_orbline/               선택 시 점선 pose로 만든 별도 TSDF
```

## 실기 구성과 저장 공간

- Pi: Ubuntu24.04/native Jazzy/Python3.12. 노트북: Ubuntu22.04/Python3.10. conda Isaac/RL 환경은 사용하지 않는다.
- 카메라는 좌우 IR640×480/Y8와 depth640×480/Z16, 15FPS, projector OFF로 요청한다. 실제 영상 크기·encoding·IR 수신률·CameraInfo와 projector 상태를 확인한다. USB2에서 depth까지 켠 조합의 실기 성능은 아직 미검증이다. USB3 케이블 사용 후 재확인 권장.
- ROS Domain67. 기존 Pi runtime `/home/pi/handheld_mapping/runtime`, adapter `/home/pi/handheld_mapping/ros2_ws/install`, DDS profile `/home/pi/handheld_mapping/live/fastdds_stereo_local.xml`을 사용한다. 이 경로가 바뀌면 `controller.remote_command`와 `pi_worker`의 기본 경로를 실제 설치와 맞춘다.
- 새 코드·카메라 로그·ORB Atlas는 `/dev/shm/rover_mapping_ID`에만 생성된다. 원본 영상은 Pi 파일로 기록하지 않고 제한된 queue에서 SSH로 노트북에 보낸다. 아직 보내지 못한 queue 자료는 단절 시 손실된다. drop은 집계하며 무누락을 보장하지 않는다.
- IR2장+depth의 압축 전 데이터는 15FPS에서 약18.4MB/s, 약1.1GB/min이다. zlib 전송/PNG 저장량은 장면에 따라 달라진다. Wi-Fi와 Pi의 압축 부하는 실제 연결 후 측정해야 한다.
- 노트북 시작 전2GB, 실행 중256MB 여유 공간 하한을 검사한다. Pi RAM의 기존 결과는 자동 삭제하지 않는다. 반복 촬영 후 노트북 저장을 확인하고 해당 앱의 이전 `/dev/shm/rover_mapping_*`만 정리한다.

## TSDF 방식과 정확성의 범위

Open3D0.19 `ScalableTSDFVolume`, voxel1cm, truncation4voxel, 유효 depth0.25~3m. voxel은 표현 해상도이며1cm 실측 정확도를 뜻하지 않는다. 가까운 바닥을 측정할 때는 이 거리 필터와 실제 카메라 지원 거리를 먼저 검토한다.

최종 `map_ID_keyframes.csv`의 camera-to-world pose와2ms 이내 source stamp의 depth만 일대일 융합한다. 중간 프레임 pose를 임의 보간하지 않는다. 따라서 초기 버전의 융합 수는 촬영 전체 프레임보다 적다. 일부 keyframe의 유효 depth가 없으면 `partial`로 표시하고, 전체 keyframe/유효 depth 없음/빈 mesh는 실패로 표시한다. 점선 지도 일부 누락도 UI에 반영한다. 서로 다른 map은 별도 파일이며 중력/바닥 정렬이나 map 자동 병합은 적용하지 않는다. 현재 adapter는 종료 때 활성 최종 map의 keyframe을 내보내므로 종료 이전 분리 지도는 최종 keyframe CSV가 없으면 재구성하지 못한다.

좌표변환은 `T_world_depth = T_world_left @ T_left_depth`, Open3D에는 그 역행렬을 전달한다. TF는 이번 실행에서 읽는다. RealSense ROS Z16 topic은 센서 자체 단위와 달리 wrapper의 `fix_depth_scale`이 mm로 변환하므로 TSDF scale은0.001m다. 센서 `depth_units`는 별도 metadata로 남긴다. [공식 wrapper 소스](https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_camera/src/base_realsense_node.cpp), [Open3D TSDF 문서](https://open3d.org/docs/release/tutorial/t_reconstruction_system/integration.html).

원본 depth에는 temporal filter/구멍 채우기/강한 평활화를 적용하지 않는다. tracking pose·최종 keyframe·실측 깊이 모두 추정 오차가 있으며 실제 벽 두께·단차 높이·loop 전후 오차로 추가 확인해야 한다. 합성 시험은 실기 정확도 검증이 아니다.

## ORB-LINE 비교

upstream: [Giannis-Alamanos/ORB-LINE-SLAM](https://github.com/Giannis-Alamanos/ORB-LINE-SLAM), commit `a351dbd1e7cfdba622891723d1b3533359f92264`, GPLv3. 기존 Pi ORB를 덮어쓰지 않는다. 구형 OpenCV include 호환 shim과 worker 종료 대기 patch만 적용한다. patch는 `experiments/handheld_mapping/orbline/shutdown_wait.patch`다.

```bash
bash tools/build_orbline.sh
bash tools/build_mapping_baseline.sh /absolute/path/to/existing/native/ORB_SLAM3
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m experiments.handheld_mapping.compare /absolute/path/to/RUN
```

창의 **ORB + TSDF → 점+선 비교**를 선택하면 기본 ORB 촬영·TSDF 저장 뒤 노트북에서 동일 좌우 IR을 ORB-LINE으로 재실행한다. point+line/LSD, scale0.5, pyramid1level, 최대400선을 사용한다. 저장 위치는 `orbline`, `dense_orbline`이다. 같은 실행 폴더에 점선 결과가 이미 있으면 덮어쓰지 않고 오류를 표시한다.

추적 OK 비율·map ID·tracking 호출 처리시간 p50/p95를 기록한다. 별도 `rover_orb3_compare`가 설치돼 있으면 같은 입력을 노트북 ORB-SLAM3로도 재생한다. Pi live 처리시간과 노트북 offline 처리시간은 직접 비교하지 않는다. 이번 line core는 빌드 메모리를 줄인 O1/native, 기존 point core는 O3/native이므로 노트북 시간 차이에도 컴파일 최적화 차이가 포함된다. GT 오차·Pi 점선15FPS·실기 모래 코스의 개선 여부는 별도 검증 대상이다. 기본값은 기존 ORB다.

이번 설치에서는 Isaac Sim stereo180쌍으로 두 C++ 실행기의 정상 종료와 최종 keyframe 저장을 확인했다. 추적 정상 비율은 둘 다100%, tracking 중앙값은 점10.21ms/점선26.63ms였다. GT 정확도 평가는 아니며 점선이 더 정확하거나 Pi에서15FPS를 유지한다는 결론은 내리지 않는다. 상세 수치와 로그는 아래 검증 기록에 있다. core·g2o·runner는 Eigen ABI 정렬이 일치하도록 같은 `-march=native` 옵션으로 빌드한다.

## 환경 재생성

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 -m venv --system-site-packages .venv-mapping
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pip install -r experiments/handheld_mapping/requirements.txt
```

native system PyQt5/OpenCV가 필요하다. Open3D/numpy/scipy는 프로젝트 venv에만 설치하며 ML/ROS system 패키지를 덮어쓰지 않는다. 노트북 native Pangolin 사본은 `outputs/handheld_mapping/native_dependencies/pangolin`에 보존한다.

## 검증 기록

계획: `docs/superpowers/plans/2026-10-13-handheld-mapping-app.md`. 실행 증거와 제한사항은 `outputs/handheld_mapping/20261013_app_validation/README.md`를 따른다. 현재 Pi/카메라 연결이 없어 SSH deploy·실기 profile·네트워크 처리량은 실행하지 않았다.

추가 서버 시험: `outputs/handheld_mapping/20261013_server_validation/README.md`. Threadripper7960X에서 같은 stereo180쌍으로 점·선 모두 정상 종료·최종 keyframe 저장과 추적100%를 확인했다. tracking 중앙값은 점8.90ms/점선19.83ms. CPU offline이며 GPU/GT/depth를 사용하지 않았다.
