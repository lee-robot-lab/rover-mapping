이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# 실제 추적 손실 구간 진단 — 2026-10-14

소스: `/home/su/rover/outputs/handheld_mapping/sessions/20261013_204443_766235`의 보존된 원본/기존 baseline 결과를 읽었다. 새로운 native run·코드 변경·장치 조작은 하지 않았다.

- baseline3604stereo쌍, tracking non-OK45frames, 약160.616–163.680s; 이후 새 map421, 이전map0와 분리.
- `loss_interval.png`:155/159.68/160.62/161.48/163.01/163.81/166.01/170.01s의 원본 IR. 단독 OpenCV ORB detector의 feature count이고 SLAM의 correspondence/inlier 수가 아니다.
- `metrics.json`: 위 samples standalone ORB1017–1145개 검출. CLAHE 후1200개지만 cap이므로 추적/정확도 개선의 증거가 아니다. 이 숫자만으로 blur/반복무늬/매칭의 문제를 배제하지 않는다.
- `input_gaps.json`:왼쪽 IR159.6837→160.6161s 사이0.93243s 공백. 첫 non-OK가 공백 직후다. depth157.9520→160.9491s 사이2.99708s 공백. 오른쪽에는 이 구간과 동일한0.93s 공백이 기록되지 않아 좌우쌍/stream균형도 조사해야 한다.
- raw right3628/left3604/depth3584, reported transport queue drop101, 마지막status source_frames3647. 마지막status는 최종센서 총수와 같다고 단정하지 않는다.

입력 공백과 추적 손실이 맞물리는 관측이며 인과 확정이 아니다. USB/카메라·ROS callback/QoS·압축·SSH/backpressure 중 어디서 잃었는지 stage별 시각/count를 추가한 새 실기 기록이 필요하다. 미래에 stereo/depth 묶음의 enqueue/누락 균형을 개선할 후보는 있으나 이번 진단에서 transport를 수정하거나 개선 효과를 측정하지 않았다.

현재 우선순위: 입력 연속성/원인계측 → IMU·encoder동시기록/clock·extrinsic·noise보정 → stereo-inertial과같은입력baseline비교 → 실제거리/단차/재방문오차로기준맵채택. ORB-LINE/CLAHE/feature숫자는동일기록 A/B로검증전기본값을바꾸지않는다.
