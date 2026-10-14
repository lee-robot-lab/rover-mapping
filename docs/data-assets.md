# 별도 데이터·실행 자산

이 저장소는 코드·설명서·작은 검증 보고서 공유본이다. 원본 workspace의 약23GB 전체를 Git에 올리지 않았다. 운영 원본은 `/home/su/rover`에 있고 공유 체크아웃과 별개다.

## Git에서 제외한 것

| 자산 | 이유 / 전달 범위 |
|---|---|
| 실제 RUN의 raw PNG·mesh/cloud·Atlas | 대용량 촬영 데이터. 필요한 RUN 전체를 별도 archive로 전달 |
| `.venv-mapping` | 머신별 Python 환경. 설치 설명서로 생성 |
| native runner·runtime·Pangolin·vocabulary | 대형/CPU·ABI 의존 자산. 기존 검증 묶음 전달 또는 pinned source 재빌드 |
| 전체 KSRC/STM firmware | 이 저장소는 확인한 제어 소스 일부만 포함. 실제 Pi checkout/firmware 별도 인계 |
| 전체 로버 mesh·시뮬레이션·경기 STL | 매핑 공유본 범위 밖. URDF는 frame/preflight 참고용 |
| SSH private key·인증 토큰·로컬 설정 | 개인 계정의 인증 환경으로 별도 관리 |

`docs/evidence/`는 작은 README/지표/회귀 로그의 사본이다. 그 문서에 등장하는 원본 PNG/CSV/큰 archive의 링크는 이 안내로 연결될 수 있다. 이는 해당 원본이 Git에 포함됐다는 뜻이 아니다. 파일 안의 과거 source 경로/hash는 provenance이며 새 clone 경로와 다를 수 있다.

## 대표 실기 RUN

원본 기준 루트: `/home/su/rover/outputs/handheld_mapping/sessions`.

| RUN | 내용 |
|---|---|
| `20261013_201134_fd506c` | camera 오류 후 보존 원본939쌍, offline18 keyframe TSDF |
| `20261013_203820_847029` | stereo1695쌍, drop13, 단일map0 |
| `20261013_204443_766235` | stereo3604쌍, drop101, map0/421 분리·입력 공백 조사 대상 |

예전 인수인계서의 날짜 RUN 경로는 예시이며 clone 직후 존재하지 않는다. 새 촬영을 하거나 담당자에게 위 RUN archive를 전달받는다.

## RUN 전달·복원

1. 촬영/후처리가 끝난 RUN 전체를 복사한다. `raw`, `capture_calibration.json`, `frames.jsonl`, `events.jsonl`, 최종 keyframe, dense summary/PLY, log를 함께 보존한다.
2. 수신자는 clone 경로의 `outputs/handheld_mapping/sessions/RUN_ID` 등에 복원한다. 다른 위치도 viewer에 경로를 명시하면 된다.
3. 기존 결과는 덮어쓰지 않는다. 재생성이 필요하면 별도 사본을 사용한다.
4. `frames.jsonl`은 PNG 상대경로, native `pairs.tsv`는 절대경로이므로 다른 PC에서는 pair index를 재생성한다.
5. 송수신 archive SHA256을 비교하고 실제 PNG/summary/PLY를 읽어 확인한다. mesh만으로 원본 재융합/공백 조사는 할 수 없다.

runtime 자산의 기대 위치는 [설치 설명서](installation.md)에 있다. 현재 private GitHub 레포에는 대용량 runtime release를 자동 게시하지 않았다.
