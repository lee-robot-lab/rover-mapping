# 표면 추적 비교와 stereo 기록 검증

## 실행 결과

- 기존 swnone/swlow/swmid/swhigh1017e의 전체 전방 PNG를 로컬로 보존했다.
- ORB seed + LK forward/backward <=1px 검사를 전체 화면과 하단 ROI에 적용했다. 각 영상쌍마다 seed를 재검출하며 이것은 pose 추정이나 장기 추적 수명 지표가 아니다.
- 약 1.23~1.36초 간격의 기존 영상에서는 하단 weighted 유지율이 무텍스처 3.6%, 저대비 0%, 중대비 6.0%, 고대비 9.6%였다. 고대비에서 하단 seed 없는 쌍은 0/35였지만 다른 조건은 9~20쌍이었다. 서로 다른 GT 주행이므로 통제된 순위나 VO 정확도로 해석하지 않는다.
- 상세 데이터: `outputs/site_sim/texture_tracking_20261017/tracking_comparison.json`, pair별 CSV.

## 새 stereo 기록

GPU0 유휴 확인 후 별도 run `stereo1017a`, observer port 8771, 고대비 표면, sim 10초, stereo 15Hz로 실행했다. 기존 8765 관찰 서버는 변경하지 않았다. 기존 실험과 새 기록은 덮어쓰지 않았다.

- 좌우 150쌍, 시각은 ns 정수이며 두 camera index가 정확히 같다.
- PNG 640×480, 별도 GT 150개, 전체 시간 9.965초.
- 실제 간격 중앙값 65ms, 최대 115ms. 요청한 15Hz가 완전한 고정 간격이라는 뜻은 아니다.
- USD에 설정한 intrinsics와 URDF optical frame 기반 extrinsics를 calibration.json으로 보존했다. 설정값 기반 보정이며 실제 D435 캘리브레이션 결과라는 뜻은 아니다.
- T_left_right의 baseline은 +X 0.05m이며 상대 회전은 identity이다.
- 같은 world.step(render=True) 이후 좌우 annotator를 읽고 실제 simulation clock으로 표기한다. renderer 자체 timestamp 증거 없이 하드웨어 수준 동기를 주장하지 않는다.

## 중요한 제한

표본 5쌍의 ORB descriptor ratio 매칭 수는 21, 11, 8, 14, 2개였다. 필터 전 세로 차 중앙값은 약 0~3.66px였다. sparse descriptor 매칭의 outlier 또는 영상 문제를 구분하는 추가 확인이 필요하다. 좌우 파일 timestamp 일치만으로 VO 투입 적합성이나 renderer 동기 정확도를 증명하지 않는다.

VO pose 추정은 이번 실행에 포함하지 않았다. 제어는 GT 그대로이며 IMU도 기록하지 않았다. 다음에는 image pair 정합·충분한 특징점·시간 association을 확인한 뒤 기존 ORB-SLAM3 stereo로 평가한다. GT를 estimator 입력으로 주입하거나 tracking 실패 때 자동으로 대신 사용하지 않는다.

## 검증과 파일

- 전체 테스트: 624 passed, 16 skipped (이번 변경 후 재실행).
- 원본 시퀀스: `outputs/site_sim/stereo_validation_20261017/stereo/mav0/cam0`, `cam1`.
- 보정·GT: 같은 stereo 폴더의 calibration.json, gt.jsonl.
- 수집 검증 수치: 같은 폴더의 validation.json.
- 서버 시험 종료 status는 timeout이며 설정한 10초 시험의 정상 상한 종료다. 대회 완주 성공이라는 의미는 아니다.
