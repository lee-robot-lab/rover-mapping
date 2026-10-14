이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../../data-assets.md)를 따른다.

# native ORB 상태·맵 분리 기전

조사 대상 source root: `/tmp/rover-native-mapping-deps/ORB_SLAM3-4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`.
실험 실행기는 현재 `outputs/handheld_mapping/orb3_comparison_build/rover_orb3_compare`와 해당 runtime 공유 라이브러리를 사용한다.

## 입력과 timestamp

- `experiments/handheld_mapping/orbline/runner.cpp:28`: `System::STEREO`로 생성한다. IMU 또는 RGB-D 모드가 아니다.
- 같은 파일 `53–60`: 좌우 grayscale PNG만 읽고 원본 `stamp_ns * 1e-9`를 `TrackStereo`에 전달한다. 별도 depth 스트림의 공백은 native ORB의 직접 입력이 아니다.
- `Tracking.cc:1820–1826`: timestamp가 이전보다 작으면 stereo에서도 새 맵을 만든다. 이 기록의 pairs.tsv는 증가 순이다.
- `Tracking.cc:1828–1853`: 1초를 넘는 timestamp jump의 reset/new-map 처리는 `mpAtlas->isInertial()` 안에 있다. 따라서 현재 non-inertial stereo에서 공백 길이만으로 강제 reset되지 않는다. 원본 실패 직전 최대 공백 0.932429932초는 1초 조건에도 못 미친다.

## 영상매칭과 motion model

- `Tracking.cc:2210–2212`: `mVelocity = T_current * inverse(T_last)`로 이전 프레임 간 상대 변환을 저장한다.
- `Tracking.cc:2870`: 다음 pose는 `mVelocity * T_last`이다. 이 경로에 timestamp 차이로 motion을 스케일하는 처리가 없다.
- `Tracking.cc:2881–2905`: stereo projection search window는 7, 매칭이 20개 미만이면 14로 확대하고 여전히 20개 미만이면 실패한다. `2946`에서는 최적화 뒤 map match가 10개 이상이어야 한다.
- `Tracking.cc:1953–1955`: motion model이 실패하면 reference keyframe tracking을 시도한다.
- `Tracking.cc:2730–2735,2778`: reference keyframe BoW match가 15개 미만이면 즉시 실패하고, 최적화 이후 map match도 10개 이상이어야 한다.
- `Tracking.cc:3030–3060`: local-map inlier의 추가 기준이 있다. 일반 stereo는 30개 이상, 최근 relocalization 직후는 50개 이상이다.

따라서 ORB keypoint를 프레임당 1000개 넘게 검출하더라도, 같은 3D map point에 대한 기하학적 inlier가 부족하면 추적은 끊긴다. 공백 중 실제 카메라 변위가 커지고 시야가 바뀌어 예측이 어긋날 수 있다는 설명은 소스와 양립하는 **가설**이다. 내부 match/inlier 수를 기록하지 않은 현재 binary로 각 실패 프레임의 정확한 판정 항목까지 확정할 수는 없다. `Fail to track local map!`는 `bOK`가 이전 단계에서 false인 경우에도 출력되므로 이 문구만으로 local-map 단계 자체의 실패를 단정해서는 안 된다 (`Tracking.cc:2125–2131`).

## relocalization과 새 맵

- `Tracking.cc:1966–1975,2155–2162`: 충분히 큰 기존 맵에서 tracking 실패하면 `RECENTLY_LOST`에 들어가 원본 timestamp를 저장한다.
- `Tracking.cc:2003–2010`: non-inertial은 relocalization을 시도하며, 첫 실패 이후 원본 timestamp로 3초를 넘고 복구하지 못하면 `LOST`가 된다. 생성자의 `time_recently_lost=5.0`은 이 stereo 분기의 3초를 바꾸지 않는다.
- `Tracking.cc:3617`: 현재 맵에 대해 candidate를 요청한다. `KeyFrameDatabase.cc:834–835`는 다른 맵의 candidate를 제외한다.
- `Tracking.cc:3648–3657`: candidate BoW match 최소 15개와 PnP RANSAC이 필요하다. `3757–3774`: 최적화 inlier 최소 50개가 있어야 relocalization이 성공한다.
- `Tracking.cc:2271–2286`: `LOST`에서 keyframe이 10개를 넘는 맵은 `CreateMapInAtlas()`를 호출한다.
- `Tracking.cc:2664–2699`: 새 맵을 만들고 state를 `NO_IMAGES_YET=0`, velocity를 false로 초기화한다. 그래서 timings.csv에는 `LOST=4` 대신 같은 호출 끝의 `state=0`이 관측될 수 있다.
- `Atlas.cc:58–76`: 이전 맵을 stored 상태로 유지하고 새 맵을 Atlas에 추가한다. 이전 맵을 삭제한 뒤 복구하는 동작이 아니다.
- `include/Tracking.h:121–128`: 상태 숫자는 `0=NO_IMAGES_YET`, `1=NOT_INITIALIZED`, `2=OK`, `3=RECENTLY_LOST`, `4=LOST`이다.

## 저장과 map_id 해석

- `runner.cpp:65–70`: timings의 `map_id`는 `Map::GetInitKFid()`이며 Atlas의 연속 map ordinal이 아니다. 예를 들어 `0/421`은 422개 맵을 뜻하지 않는다.
- `System.cc:538–550`: mapping/loop-closing worker 종료를 확인한 다음 Atlas를 저장한다.
- `System.cc:1408–1438`: Atlas 전체를 `atlas.osa`로 직렬화한다.
- `Atlas.cc:316–332`: 비어 있지 않은 valid map을 보존하며 빈 맵을 제거한다. 맵 분리가 생겨도 마지막 맵만 저장하는 구조가 아니다.
- `runner.cpp:85–110`: 관측했던 valid map들의 최종 keyframe을 별도 CSV로 출력한다. 서로 다른 map 좌표는 별도 gauge이므로 이를 같은 원점으로 단순 합쳐서는 안 된다.

## 동일 재생의 변동 가능성

- `System.cc:202,219`: LocalMapping과 LoopClosing을 별도 thread로 실행한다.
- `Tracking.cc:3103,3158,3187–3203`: mapper의 idle 상태와 queue 길이가 keyframe 추가 판정에 관여한다. `LocalMapping.cc:154,288`에서는 BA가 interrupt flag에 의해 중단될 수 있다.
- 동일 image/timestamp/settings 재생의 state·map 결과가 달라진 것은 관측 사실이다. 비동기 scheduling과 keyframe/BA 경로는 그 차이를 설명할 수 있는 소스상 요인이다. 각 thread schedule을 계측하지 않았으므로 이를 유일한 원인으로 확정하지 않는다.
- `runner.cpp:49–74`는 recorded timestamp 간격만큼 기다리지 않고 파일을 최대 속도로 재생한다. 특징 수를 늘려 tracking 처리시간을 바꾸면 mapping thread와의 실행 순서도 달라질 수 있다. 단회 설정 비교의 차이를 순수한 특징 수 효과로 분리할 수 없고, 이 offline elapsed time은 실제 capture FPS나 live latency가 아니다.

## 허용 가능한 다음 조치

1. 입력 공백 원인을 수집·전송·기록 경계의 timestamp와 drop reason으로 해결한다. image, exposure, motion blur, stereo sync와 실제 matched/inlier 수를 함께 기록해야 feature shortage와 correspondence shortage를 구분할 수 있다.
2. 설정 비교는 같은 원본·timestamp에서 여러 번 돌리고 비정상 프레임 수·실패 구간·최종 valid map 수·처리시간을 함께 본다. 설정별 1회 결과만으로 개선을 확정하지 않는다.
3. status가 OK가 아닌 pose를 정상 map frame으로 사용하지 않고 맵이 갈라졌음을 결과에 드러낸다. threshold 연장이나 motion 모델 변경은 별도 native 소스 계측·회귀 실험 대상으로 제안할 뿐 현재 production에 적용하지 않는다.
4. GT 검증에는 외부 측정 pose와 시간 정렬을 사용해 ATE/RPE, metric scale, loop consistency와 실패 복구를 평가한다. map이 갈라진 실행의 전체 경로 정확도를 하나의 rigid alignment로 산출하지 않는다. 추적 지속성과 맵 수 감소는 GT 정확도 증거가 아니다.
