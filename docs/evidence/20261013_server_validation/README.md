이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# ORB-LINE 서버 실제 재생 결과 — 2026-10-13

서버 `hpi-lab` / `TRX50-AI-TOP`, AMD Ryzen Threadripper7960X24코어에서 두 C++ 실행기를 실제 빌드·실행했다. 작업 전용 컨테이너 exit0, 두 runner 모두180프레임 처리와 최종 keyframe 저장을 확인했다. 원본·실행 파일은 서버 `/home/sumee/gyusa/runs/handheld_mapping/20261013_032754_orbline`에 보존했다.

| 동일180프레임 | 추적 정상 비율 | tracking 중앙값 | tracking p95 | 최종 keyframe | map |
|---|---:|---:|---:|---:|---:|
| ORB-SLAM3 점 | 100% | 8.90ms | 10.27ms | 39 | 0 |
| ORB-LINE 점+선 | 100% | 19.83ms | 22.54ms | 46 | 0 |

- 최종 요약: [comparison.json (별도 자료)](../../data-assets.md).
- 원본 측정: `results/input/{orbline,orb_offline}/timings.csv`와 `map_0_keyframes.csv`.
- 완료 증거: 각 `runner_complete.json`, `results/container_state.txt`, `results/container.log`, `results/exitcode.txt`.
- 환경/빌드: `results/environment.txt`, `results/build_flags/`, `results/*build.log`, `container_manifest.txt`.
- 전체 build+재생 container 시간132.14초 (03:30:15~03:32:27KST). 개별 tracking 시간과는 다른 값이다.

## 입력·범위

이전 노트북 검증과 같은 Isaac Sim stereo180쌍을 사용했다. calibration·frames index·360PNG의 SHA256 목록이 로컬과 서버에서 모두 일치했다 (`local_input.sha256`, `server_input_normalized.sha256`, `cmp` exit0). 새 시뮬레이션이나 실시간 카메라 촬영은 하지 않았다. depth와 GT를 사용하지 않았으므로 TSDF·실측 오차 평가는 포함하지 않는다.

표의 시간은 native `TrackStereo` 호출의 경과 시간이다. 이미지 파일 읽기·저장·vocabulary load·전체 비동기 mapping 비용과 종료 대기를 모두 포함한 end-to-end FPS가 아니다. Pi15FPS나 카메라→화면 latency를 의미하지 않는다. 두 모드가 이 짧은 입력에서 모두100%를 보였으므로 점선의 정확성·안정성 우위는 아직 검증되지 않았다.

GPU 미사용. OpenMP2/OpenBLAS1, SLAM 자체 mapping thread는 활성화했다. rootless Docker CPU quota가 지원되지 않아 `--cpus` 대신 build parallel4/2와 위 thread 환경변수를 사용했다. 이번 container의 line core/g2o는 O1/native, 기존 point core는 O3/native라서 시간 차이에 compiler 최적화 차이도 포함된다. Ubuntu24.04/OpenCV4.6/Eigen3.4/GCC13/Python3.12 system, conda 미사용. 정확한 버전은 환경 파일에 보존했다.

## 안전한 최종 자세 저장

기존 image `rover-orbslam3-jazzy:4452a3c4`의 point core는 `Shutdown()` worker 대기가 주석 처리돼 있었다. 작업 전용 컨테이너 안에서만30초 제한 대기를 복구하고 System.cc를 다시 컴파일했다. `results/point_shutdown.patch`가 변경 증거다. 기존 image나 다른 컨테이너는 수정하지 않았다. line core는 앞선 upstream shutdown 대기 patch를 그대로 사용했다.

서버 실행 때문에 `compare.summarize_timings`의 고정 문구 `offline laptop`을 `offline ... on executing machine`으로 일반화했다. 실제 tracking CSV는 변경하지 않았고 요약 metadata만 다시 생성했다 (`refresh_summary.py`, `results/summary_refresh.log`). 최초 출력은 `results/comparison.log`에 그대로 남겼다. 관련 compare 테스트3개와 pyflakes 통과.

결과 archive SHA256: `d17cf1d9ee4322d1b325c8c9b41b801b781aa47a852355c3cd46810612c16e10`. 서버와 노트북의 archive hash가 일치한다. 실행 스크립트는 `run_container.sh`, patch 적용은 `patch_point_shutdown.py`다. 새 run 폴더에 업로드 archive를 풀고 기존 image를 `/work`로 mount하면 동일 절차를 재현할 수 있다. 기존 run을 다시 덮어쓰는 용도는 아니다.
