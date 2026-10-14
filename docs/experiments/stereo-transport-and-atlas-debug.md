# Stereo 전송 누락과 Atlas 병합 조사

2026-10-09. 로컬 Ubuntu22.04/x86/Humble, `/usr/bin/python3`3.10.12, conda unset. 실제 Pi24.04/ARM/Jazzy·D43535°·조종 시험은 미실행이다. 원본 영상·source stamp·QoS·판정 기준과 ORB 추적/BA 알고리즘을 유지했다.

## 같은 호스트의 영상 누락

Fast DDS2.6.12, `rmw_fastrtps_cpp`에서 640×480 mono8·30Hz를 별도 publisher와 두 consumer로 시험했다. publisher는 Python 픽셀별 변환 병목을 제외한 array를 사용한다. 각 영상에 넣은 진단 sequence와 원본 source stamp로 수신 여부를 확인했다. 합성 입력이며 실제 촬영 영상이나 실기 정확도 자료가 아니다.

| 조건, 300쌍 발행 | sink 좌/우 | probe 좌/우 | probe exact 후보 |
|---|---:|---:|---:|
| 기본 transport | 290/300 | 276/300 | 276 |
| 명시적 SHM512 KiB + UDP | 286/300 | 282/300 | 282 |
| 같은 XML에서 SHM만32 MiB | 300/300 | 300/300 | 300 |

대조 XML 사이의 유일한 차이는 `segment_size`다. 발행 주기/메시지/QoS/소비자는 같고, 실험 중 UDP `RcvbufErrors`/`InErrors` 증가도0이었다. pair publish 최댓값은 약1.1~1.35ms였다. 이 조건에서 SHM 용량 부족이 영상 누락에 영향을 주는 것을 확인했으며, 모든 환경에서 손실이 없어졌다는 뜻은 아니다.

[Fast DDS 공식 SHM 문서](https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/transport/shared_memory/shared_memory.html)는 기본 segment512 KiB와 메시지에 비해 작은 segment의 위험을 설명한다. 적용 파일은 `ros2_ws/src/orbslam3_ros2/config/fastdds_stereo_local.xml`이다. participant별32 MiB SHM과 UDP discovery를 사용한다.

카메라·ORB·recorder·bridge **각 프로세스를 시작하기 전 모든 셸**에서:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix orbslam3_ros2)/share/orbslam3_ros2/config/fastdds_stereo_local.xml"
```

이미 실행 중인 카메라 publisher에는 consumer 셸의 변경이 소급되지 않는다. 실행한 XML을 run에 복사해 보존한다. 다른 RMW에는 이 XML이 적용되지 않는다. 커널 설정·장치·Wi-Fi·SPI를 변경하지 않았다.

제품 XML을 사용한 실제 크기30초 기능 시험은 정상 입력 좌900/우899·exact899, 약30Hz로 `capture_ready=true`였다. 양쪽 callback의 수집 종료 경계 때문에 한 프레임 차이가 있으며 이 수치로 무손실이라고 주장하지 않는다. profile을 한 번 바꿨다가 되돌린 입력은 변경2회로 `capture_ready=false`였다. 기존64×48 기능 시험과 구분한다.

자료는 `outputs/headless_mapping/20261009_transport_debug/`의 `baseline_both`, `control_shm512`, `test_shm32`, `full_profile32`다. 재현 driver와 대조 XML도 같은 폴더에 보존했다. source-age 수 ms는 이 로컬 합성 부하의 결과이며 Pi/USB/네트워크 성능이 아니다.

같은 제품 XML로60초·1800쌍을 발행한 `long32`에서는 sink/probe 둘 다 좌1800/우1800이었다. sink는 전체 sequence/source stamp를 보존했고, probe의 exact 후보1000은 원래 마지막1000개 window 제한이다. 동시 API 재생 부하에서도 sink 최대 age4.61ms, pair publish 최대1.19ms, UDP 오류 증가0이었다.

## 원본 파일 재생의 Python 병목

기존 `Image.data=image.tobytes()`는 ROS Python setter에서 픽셀별 검사를 거쳐 약19.67ms/image였다. stereo 두 장만으로30Hz의33.3ms 예산을 넘는다. `array('B', image.tobytes())`로 바꾸면 같은 setter 계측은 약0.0054ms/image였다. PNG decode·전송·ORB 처리 비용을 포함한 수치가 아니다.

현재 재생 도구는 uint8 array를 전달한다. 해상도 축소·압축·source stamp 재작성·frame 생략은 하지 않는다. 이전 bytes 구현은 별도 진단 사본에서만 재생하고 현재 구현과 동일한 원본300쌍의 실제 MCAP payload/촬영 시각을 비교했다. 양쪽 모두 좌우300장 원본과 완전히 같았고 기록 receipt 구간은 기존12.8505초→수정9.9684초였다. 이 조건에서30Hz 재생을 확인했다. 결과는 `replay_payload/result.json`에 보존한다. ORB까지30Hz 처리했다는 뜻은 아니다. 첫 실기 준비 판정을 느슨하게 바꾸지 않았다.

## Atlas 병합 종료 조사

이전 `20261009_replay/load04`는 병합 중 SIGSEGV가 한 번 발생했고 스택/core가 남지 않았다. 최초 map 생성의 C++14/17 Eigen ABI abort와 구분한다. map 표시 identity 수정이 이 SIGSEGV를 고쳤다고 주장하지 않는다.

현재 adapter와 valid `stereo05` Atlas로 GDB 아래에서6회 반복했다. 각360쌍, 시작 index0/0/90/180/300/0,0.5배속이다. 모두 load·추적·정상 종료·Atlas 저장·MCAP 기록 검사를 통과했다. 이 중5회는 `Merge finished!` 로그까지 있고, index300의1회에는 병합 완료 로그가 없다. 반복 성공만으로 예전 SIGSEGV의 원인을 확정하지 않는다.

자료는 `outputs/headless_mapping/20261009_merge_debug/merge_debug_validation.json`과 각 `loadgdb*/node.log`다. GDB driver는 SIGSEGV 시 전체 thread backtrace와 core를 저장한다. 모든 replay는 기존 cam0/cam1 CSV/PNG만 읽으며 GT/depth/정답 경로는 읽지 않는다. 별도 실제 주행 Atlas reuse는 계속 미검증이다.

### 실패 당시 Atlas에서 재현한 원인

같은 `load04/atlas_input.osa`(SHA256 `ee595f54a383dc1b0b979eef68ab1df5fa7f168d53bbc7c2a7d09de2ffc62c8b`)를 공식 `System::TrackStereo` API에 직접 넣었다. ROS adapter/지도 snapshot 없이 원본240쌍을0.5/1/0.5배속으로 재생했으며 세 번째에 동일한 병합 단계에서 SIGSEGV가 재현됐다. 첫 진단 시도는 Pangolin loader 경로 누락으로 실행되지 않았고 `old_api01..04`에 보존했다. 환경을 바로잡은 실제 재생은 `old_api_env01..03`이다.

`old_api_env03/crash.core`와 전체 thread stack을 확보했다. tracking thread가 `ORBmatcher::SearchByProjection`의 `F.mvScaleFactors[nPredictedLevel]`에서 종료됐고 병합 thread는 local BA 중이었다. dump에서 frame410, scale levels8, 해당 loaded point1694의 `mbTrackInView=85`(정상 bool 표현이 아님), `mnTrackScaleLevel=1769473`, `mnLastFrameSeen=410`을 확인했다. 8개 배열 밖을 읽은 명령도 disassembly로 확인했다. 당시 원본 공유 라이브러리는 `original-library/`에 보존했다.

공식 `MapPoint::serialize`는 tracking cache를 저장하지 않고 default constructor도 view flags/scale levels를 초기화하지 않는다. motion-model outlier 처리 뒤 같은 frame seen으로 frustum 재계산을 건너뛰는 경로에서 그 값이 매칭에 사용될 수 있다. `0x55`로 채운 실제 placement-new 생성자 검사에서 `view=85,85 scale=1431655765,1431655765`로 실패했다. 문제는 raw 영상/ROS 전송이나 표시 map ID가 아니라 역직렬화 객체의 초기화 누락이다.

수정은 `patches/orbslam3_atlas_load_cache.patch`의 default constructor 네 값 초기화뿐이다. 새 검사에서는 `view=0,0 scale=0,0`이며 영상을 생략하거나 scale 범위 gate를 추가하지 않았다. 추적/BA/매칭 판정이나 Atlas 직렬화 형식도 바꾸지 않았다. CMake/source marker와 실제 library link symbol을 요구하고 `test_atlas_load_cache`가 재사용 메모리 세 pattern에서 이를 검사한다.

최초 `load04`에는 stack이 없어 그 한 번의 고장을 개별적으로 증명할 수는 없다. 이번에 같은 입력/단계의 고장을 재현해 원인과 수정 전 실패를 확보한 것이다. 수정 후 반복과 ROS end-to-end 결과는 아래 검증 기록에 추가한다.

### 수정 후 검증

- 같은 실패 Atlas와 원본240쌍을0.5/1배속 번갈아6회 재생했다. `fixed_api_results.json`의6회 모두 정상 종료·`Merge finished!`·240/240 tracking OK·Atlas 저장이다.
- 수정 DSO/adapter로 ROS 재생2회(`20261009_merge_fixed/loadgdb01,02`, 각360쌍/0.5배속)를 실행했다. 각360/360 poses, MCAP 좌360/우360, 추적/변환 실패0, load·merge·Atlas/PLY 저장·최종 리포트 `evidence_complete=true`였다.
- MCAP을 실제 reader로 다시 읽어 좌우 영상과 camera/base pose 전부 원본 source stamp와 일치함을 확인했다. pose는 finite이고 `orb_map_60`→`orb_map_0`, 이전 map0 alignment는 정확히 유지됐다. `bag_audit_summary.json`에 보존했다. 이 발행률은 위치 정확도가 아니다.
- 현재 gtest13개/6targets와 Python mapping65개 통과. 새 gtest는 수정 전 원본 DSO를 process-local preload하면 실패하고 수정 DSO에서는 통과한다. 전체 pytest는 기존 `test_mesh_inertia.py`, `test_mesh_preparation.py`가 `trimesh` 미설치로 collection error2개를 낸다. 이를 mapping 통과와 구분한다.
- adapter/native DSO를 재빌드하고 두 patch marker의 실제 symbol, 설치 XML checksum 일치, 원본 URDF checksum 유지를 확인했다. 기존 compiler 경고는 남아 있어 warning-free build로 주장하지 않는다.

실제 Pi native Jazzy 재빌드·35° D435·조종/SPI·센서 동기화·별도 실제 주행 reuse·절대 위치 정확도는 아직 검증하지 않았다. 처음 실기에서는 기존 계획대로 정지30초 입력 검사 후 새 지도를 만든다.
