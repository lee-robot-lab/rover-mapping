# 고대비 표면 ORB-SLAM3 stereo 평가

## 조건과 실행

- 2026-10-17, stereo1017a에서 저장한 고대비 stereo 150쌍(약 10초), PNG 640×480, 설정 15Hz, baseline 5cm.
- 기존 point baseline `20261015_tracking_recovery/baseline/rover_orb3_compare`와 동봉 runtime 사용. CPU offline 실행, IMU 없음. ORB-SLAM3 stereo SLAM이므로 순수 frame-to-frame VO와는 다르며 재지역화·map 재생성을 포함한다.
- 기존 기본값 ORB 1200개 상한, FAST 20/7. 추정기에는 좌우 이미지·카메라 보정·timestamp만 제공. GT 입력 없음.
- 첫 빠른 파일 재생은 실행 확인용(vo_high_20261017a). 최종 평가는 초기화 완료 후 원래 촬영 시각 간격으로 stdin 재생한 vo_high_20261017b.
- runtime/library/settings SHA-256은 b/input_manifest.json에 기록. 서버의 진행 작업과 제어 pose는 변경하지 않았다.

## 결과

- 150개 모두 처리, tracking OK 134/150 = 89.33%.
- tracking을 얻은 map 원점 3개. 처음 두 map은 각 1개 유효 frame만 남았으므로 정확도 근거로 사용하지 않는다.
- 주된 map 2: 132개 유효 pose, 시뮬레이션 약 1.12~10.015초, GT 이동 길이 약 0.691m.
- 이 map의 최초 유효 pose 하나로 SE(3) 정합. 전체 궤적 fitting/scale fitting 없음. camera pose를 기록된 T_base_left_optical 역변환으로 base pose에 변환해 GT base pose와 비교했다.
- 주된 map 위치 RMSE 0.1446m, 최대 오차 0.2474m, 끝점 오차 0.2277m. 전체 자세 회전 오차 RMSE 6.97°(yaw만의 오차가 아님).
- 오차는 tracking OK 구간에서만 계산. 손실 구간을 제외했다는 점을 추적률·map 개수와 함께 봐야 한다. 재초기화된 map들을 하나의 정확한 전역 궤적으로 합쳤다는 뜻이 아니다.

## 판단

고대비 영상에서 ORB stereo의 초기화·위치 출력은 가능하다. 하지만 69cm 이동 구간에 23cm 끝점 오차와 초기 재설정, 중간 재지역화가 발생해 현재 GT를 대체할 준비는 안 됐다. 특징점 유지율이 충분해 보이더라도 위치 정확도를 보장하지 않는 실례다.

원인이 표면이라고 단정하지 않는다. 기존 수집 검증의 적은 좌우 match 수/세로 오차를 포함해 renderer 영상 시각 association, 실제 렌더된 intrinsics·좌우 extrinsics, motion/image quality를 점검해야 한다. 고대비 표면은 개발용 기준으로 유지하고 이 문제부터 확인한다.

## 산출물

- `outputs/site_sim/vo_high_20261017b/evaluation.json`: map별 오차·tracking·처리 시간.
- `live_poses.csv`, `timings.csv`: 원본 추정 pose와 tracking 상태.
- `pose_errors.csv`: 평가 시각별 위치·자세 오차.
- `trajectory_comparison.png`: 가장 긴 map 구간의 GT/VO 비교.
- `orb.yaml`, `input_manifest.json`, `execution.json`, `run.log`: 입력·실행 증거.
- `experiments/site_sim/vo_evaluation.py`: 평가기, 초기 강체 정합이 scale 오차를 숨기지 않는 회귀 테스트 포함.
