#!/usr/bin/env bash
# ================================================================
# reference_mapping.sh
# 설명: 원본 촬영을 보존하며 노트북에서 depth 전역 재구성 결과를 새 폴더에 만든다.
# 사용법: bash tools/reference_mapping.sh RUN OUTPUT --backend orb_offline --map-id 0 [--color]
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
exec "$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.global_reconstruct "$@"
