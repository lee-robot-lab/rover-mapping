# External ORB–IMU attitude fusion implementation plan

**Goal:** 저장 실기 기록으로 Stereo ORB의 위치를 유지하면서 외부 IMU 자세 보조의 실행 가능성을 비교한다.

**Architecture:** 별도 CLI가 기존 `aligned_inputs`의 실험용 공통 시간을 사용한다. body축 gyro를 SO(3)에서 적분하고, 정지 구간에서 설정한 map별 중력 기준으로 가속도 피드백 및 ORB 회전 피드백을 적용한다. ORB map 변경·큰 pose 점프·IMU 공백은 명시적으로 재기준 설정한다. 추적 손실은 최대0.3초 회전 예측만 제공하고 위치 valid=false를 유지한다. 원본·ORB 내부·TSDF는 수정하지 않는다. 시간/외부변환 검증 false와 정확도 미평가를 명시한다.

**Stack:** mapping Python3.10 venv, NumPy, SciPy Rotation, matplotlib Agg. Conda 미사용.

- [x] 필터의 회전축, 중력 방향, 공백/추적손실/지도경계 처리 검사를 먼저 작성한다.
- [x] `attitude_fusion.py`: 상보 필터, 저장 기록 replay, 별도 CSV/JSON/PNG 비교 출력.
- [x] 실제 기록 `20261016_004946_f86cc9`로 replay하고 원본 위치 보존·rotation 유효성·fallback 표시를 검사한다.
- [x] Stereo / 기존 관성 실행 / 외부 자세 보조의 범위와 사용법을 문서화한다. 서로 다른 map 좌표의 직접 오차 비교는 하지 않는다.
- [x] 관련 회귀 검사 후 runtime에 새 파일만 배치하고 결과 링크를 제공한다.

검증: 관련 검사66개 통과. 실제2,449행에서 원본 위치·hash 불변, SO(3), 무효 위치 NaN, gyro-only 기준 map 보존을 확인했다. 독립 리뷰 P2(map provenance)를 수정 후 재리뷰에서 중요한 추가 문제 없음. 결과는 노트북 `outputs/handheld_mapping/20261016_external_attitude_v2`에 보존했다. 동작 변경은 offline 실험으로 제한한다.
