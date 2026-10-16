#!/usr/bin/env bash
# ================================================================
# attitude_fusion.sh
# 설명: 저장 ORB·IMU를 외부 자세 필터로 비교하고 별도 결과를 생성한다.
# 사용법: bash tools/attitude_fusion.sh RUN [--output NEW_DIR] [--segment 0]
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
exec "$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.attitude_fusion "$@"
