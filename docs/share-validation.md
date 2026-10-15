# 공유본 검증 — 2026-10-15

## Stereo 추적 복구 추가 검증

- 공유본 전체 `pytest -q`: **219 passed, 1 skipped (34.54초)**. Git에서 제외한 선택 native 자산 검사는 skip이다. [실행 로그](evidence/tracking-recovery-pytest.log).
- 원본 매핑 범위219개 통과, pyflakes·bash 문법·Git whitespace 검사 통과. FIFO backlog 중 화면 갱신과 입력 지연에 따른 stale 표시는 독립 review 후 회귀 검사했다.
- 실제 native 비교: 성장한 지도에서4.07초 blank 뒤 기존map0으로 즉시 복귀. 설정15초에서16.07초 blank는15.07초에 reset하며 기본 설정은3.07초에 reset한다. blank 구간 유효 pose0.
- 새 core/runner를 작성 머신에 빌드·설치하고 marker 로딩·hash를 확인했다. 새 PC native 전체 설치나 현재 Pi 실기 완료를 뜻하지 않는다. [변경·재생·한계](experiments/2026-10-15-tracking-recovery.md).

## IMU 동시 기록 추가 검증

- 최신 공유본 전체 `pytest -q`: **205 passed, 1 skipped (14.69초)**. 선택 native 실행기 검사는 설치 자산이 없어 skip이다.
- 원본의 매핑 범위 검사: **205 passed (18.89초)**. source manifest 111개 hash, 변경 문서 링크49개·shell block28개 확인.
- 실제 Pi: 보정 IMU와 D435 각122프레임 동시 저장·정상 종료, IMU957개 고유 샘플, 영상/tee 누락0, ORB/TSDF 생성 확인. 짧은 정지 연결 시험이며 주행 정확도 검증은 아니다.
- 시계 동기화·관성 융합은 false로 유지한다. 원본·장비별 보정 JSON은 Git에서 제외했다. [사용법과 검증 범위](experiments/2026-10-14-imu-co-recording.md).

아래는 최초 공유본 생성 때의 검증 기록이다.

별도 공유 체크아웃 `/home/su/rover-mapping`에서 원본의 Python 환경을 사용해 검증한다. 소스·테스트·wrapper는 원본 hash와 대조하고, native 바이너리/촬영 데이터가 없는 공유본의 결과를 따로 기록한다. 새 PC의 전체 native/RealSense 설치 및 현재 Pi 실기는 이 검증에 포함되지 않는다.

## 실제 결과

| 검증 | 결과 |
|---|---|
| 공유본 전체 `pytest -q` | **173 passed, 1 skipped (10.04초)** |
| optional native load 검사 | 바이너리를 Git에서 제외해 1개 skip. 실제 native 재빌드 성공으로 해석하지 않음 |
| 합성 Session | 8개 stereo/depth frame, 정상 종료, TSDF map1개 저장·viewer 선택 확인 |
| 합성 terrain | terrain.npz/summary.json/preview.png 생성 |
| 문서 검사 | Markdown50개, 로컬 링크115개, shell 예제67개 검사 통과 |
| 코드/테스트 복사 | hash105개 확인. 운영 소스·wrapper는 원본과 동일 |
| 정적 검사 | pyflakes·wrapper bash 문법 검사 통과 |
| upstream patch | 고정 ORB-SLAM3 stock source의 C++14/shutdown/Atlas patch3개 dry-run 통과 |

처음 공유본에서 170 passed/1 skipped/2 failed였다. 두 실패는 테스트가 원래 머신의 native 실행기 설치를 가정한 데서 생겼다. 공유본 테스트만 수정해 GUI는 native 있음/없음 두 조건을 확인하고, baseline 재사용 테스트는 필요한 runner 파일을 fixture로 준비하도록 했다. 운영 코드는 바꾸지 않았다. 원본/shared hash와 변경 이유는 [source-snapshot.json](source-snapshot.json)에 있다.

회귀 로그는 [share-pytest.log](evidence/share-pytest.log), 문서 검증은 [JSON](evidence/share-document-validation.json)에 있다. 원본 workspace의 173통과와 공유본의 173통과+1skip은 별도 실행이다. 공유본 GUI 테스트의 조건을 한 개 추가했으므로 수집 case 수가 원본과 다르다.

## 재현 명령

```bash
KSRC_REVIEW_ROOT="$PWD/teleop/ksrc_snapshot" \
QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m pytest -q
```

작성 머신에서는 clone에 venv를 중복 설치하지 않고 `/home/su/rover/.venv-mapping/bin/python`을 명시해 실행했다. Python3.10/PyQt5/OpenCV/Open3D 의존성이 이미 갖춰진 환경이다. 새 PC의 apt/pip 전체 설치·native 전체 컴파일·현 Pi/USB/무선 실기·모터 조종은 이번 검증에 포함하지 않았다.
