#!/usr/bin/env bash
# ================================================================
# handheld_mapping.sh
# 설명: conda/user-site를 섞지 않고 노트북 D435 매핑 제어창을 연다.
# 사용법: bash tools/handheld_mapping.sh [--demo]
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
unset QT_QPA_PLATFORM_PLUGIN_PATH
exec "$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.app "$@"
