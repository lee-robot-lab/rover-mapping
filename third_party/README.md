# 외부 의존성·버전

외부 전체 소스/바이너리/vocabulary는 Git에 넣지 않는다. `external/`에 지정 버전으로 설치한다. private 저장소 여부와 별개로 각 upstream의 원저작권·라이선스를 보존한다.

| 프로젝트 | 기준 | 사용 |
|---|---|---|
| [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` | point stereo core. upstream GPLv3 |
| [ORB-LINE-SLAM](https://github.com/Giannis-Alamanos/ORB-LINE-SLAM) | `a351dbd1e7cfdba622891723d1b3533359f92264` | 선택적 point+line 및 현재 vocabulary 경로. upstream GPLv3 |
| [Pangolin](https://github.com/stevenlovegrove/Pangolin) | v0.8, tag `aff6883c83f3fd7e8268a9715e84266c42e2efe3` | native 의존성. 설치 package version0.8 확인 |
| KSRC 제어 소스 일부 | `4bd158b1125cee32c5851f74ecc0c8f3d71013b1` 기준 | `teleop/ksrc_snapshot/ORIGIN.md` 참고 |

보존한 GPL 문구는 [ORB-SLAM3](licenses/ORB-SLAM3-GPL-3.0.txt), [ORB-LINE](licenses/ORB-LINE-SLAM-GPL-3.0.txt)에 있다. `ros2_ws/src/orbslam3_ros2/package.xml`은 GPL-3.0-only를 명시한다. 해당 소스의 기존 표시는 유지한다.

patch 위치:

- `third_party/patches/orbslam3-cxx14.patch`: pinned point upstream의 C++14 build 설정. stock 파일 dry-run 확인.
- `ros2_ws/src/orbslam3_ros2/patches/orbslam3_shutdown_wait.patch`: bounded 종료 대기와 adapter marker.
- `ros2_ws/src/orbslam3_ros2/patches/orbslam3_atlas_load_cache.patch`: Atlas load cache 초기화.
- `experiments/handheld_mapping/orbline/shutdown_wait.patch`: point+line 종료 대기.
- `ros2_ws/src/orbslam3_ros2/patches/ksrc_telemetry_tee.patch`: 선택적 telemetry tee 초안. 현재 Qt에 자동 연결되지 않는다.

core/g2o/runner의 Eigen ABI·아키텍처 옵션을 맞춘다. 현재 recipe의 `-march=native` 결과를 다른 CPU에서 그대로 쓸 수 있다고 가정하지 않는다. native 설치와 검증 한계는 [설치 설명서](../docs/installation.md)에 있다.
