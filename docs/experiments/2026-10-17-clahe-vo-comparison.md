# 동일 고대비 stereo에서 CLAHE 전후 비교

## 통제한 항목

- 원본 stereo1017a 150쌍, 동일 settings YAML(바이트 동일), 동일 ORB binary/runtime, 원래 simulation timestamp 간격으로 1x 재생.
- 좌우 모두 grayscale 이후 `cv2.createCLAHE(clipLimit=3,tileGridSize=(8,8))`. 원본은 보존하고 처리 PNG를 별도 폴더에 기록했다. 추가 보정·resize 없음.
- GT는 추정기에 전달하지 않고 평가에만 사용. 결과마다 map별 최초 pose 한 개의 SE(3) 정합만 사용하며 scale fitting 없음.
- 한 번씩의 실행으로 ORB 실행 변동까지 통제한 반복 실험은 아니다.

## 결과

| 항목 | raw: vo_high_20261017b | CLAHE: vo_high_clahe_20261017a |
|---|---:|---:|
| 처리 영상쌍 | 150 | 150 |
| tracking OK | 134 (89.3%) | 100 (66.7%) |
| 추적을 얻은 map 수 | 3 | 5 |
| 최장 연속 nontracking frame 수 | 8 | 47 |
| 가장 긴 map의 유효 pose 수 | 132 | 86 |
| 해당 map 시간 | 1.12~10.015s | 4.345~10.015s |
| 해당 구간 GT 이동 길이 | 0.691m | 0.446m |
| 해당 구간 위치 RMSE | 14.46cm | 2.13cm |
| 해당 구간 끝점 오차 | 22.77cm | 2.13cm |
| 해당 구간 최대 오차 | 24.74cm | 3.56cm |
| ORB 처리 median / p95 | 7.67 / 9.08ms | 11.92 / 14.09ms |

중요: 마지막 map의 유효 구간 길이가 다르므로 RMSE 감소를 같은 전체 경로의 정확도 향상으로 단정할 수 없다. CLAHE는 약 1.12~4.345초 사이 추적이 크게 끊겼으며 이 구간은 pose 오차에 들어가지 않는다. 전체 성공률은 악화됐다. 짧은 구간의 낮은 오차만 보고 기본 적용을 결정하지 않는다.

## 처리 시간

이 노트북 CPU에서 OpenCV 4.5.4, 1 thread, 640×480 좌우 한 쌍, 20회 warmup 후 300회 측정:

- CLAHE median 1.975ms, p95 2.067ms.
- capture, 통신, 이미지 읽기·변환, ORB, 나머지 로봇 처리와 OS 지연은 이 값에서 제외됐다.
- 실기 컴퓨터·Pi에서의 성능은 측정하지 않았다. 15Hz(66.7ms)나 30Hz(33.3ms)의 end-to-end 보장은 아니다.
- ORB 처리 시간도 CLAHE 영상에서는 특징점 증가 등으로 길어졌다. 별도 전처리 시간만 비교하지 말아야 한다.

## 파일

- 처리 이미지와 benchmark: outputs/site_sim/stereo_clahe_20261017a/preprocessing.json 및 mav0/cam0,cam1.
- 추정과 평가: outputs/site_sim/vo_high_clahe_20261017a/live_poses.csv, evaluation.json, pose_errors.csv, execution.json, input_manifest.json, run.log.
- 비교 기준: outputs/site_sim/vo_high_20261017b.

다음 확인은 공통 유효 시간 구간의 오차와 추적 손실 원인이다. CLAHE가 유일한 원인이라는 결론이나 실기 기본 적용 변경은 아직 하지 않았다. 서버·실기 코드는 변경하지 않았다.

## 공통 유효 구간 추가 비교

두 실행이 모두 tracking OK인 동일 86개 timestamp(4.345~10.015s)를 선택하고 두 결과 모두 이 구간의 첫 GT pose 하나에서 초기 SE(3) 정합을 다시 적용했다. scale fitting은 없다.

- raw RMSE 15.26cm, 끝점 19.98cm.
- CLAHE RMSE 2.13cm, 끝점 2.13cm.

이 공통 구간에서는 CLAHE 실행의 정확도가 좋았다. 하지만 앞선 약 3.115초 연속 추적 손실은 이 비교에서 제외됐다. 따라서 전체 실험의 가용성까지 좋아졌다는 뜻은 아니다. 원자료는 vo_high_clahe_20261017a/common_valid_comparison.json.
