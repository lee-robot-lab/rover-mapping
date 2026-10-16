#!/usr/bin/env bash
# ================================================================
# edit_reference_map.sh
# 설명: 중력 정렬·영역 선택·출발 위치·반복 지도 병합을 위한 노트북 편집 창을 연다.
# 사용법: bash tools/edit_reference_map.sh RUN_OR_RESULT [--map-id 0]
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
cd "$root"
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
unset QT_QPA_PLATFORM_PLUGIN_PATH
exec "$root/.venv-mapping/bin/python" -m experiments.handheld_mapping.reference_editor "$@"
