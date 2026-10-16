#!/usr/bin/env bash
# ================================================================
# plan_site_mission.sh
# 설명: 팀원 STL 또는 저장 임무의 대표점·세 경로 편집 창을 연다.
# 사용법: bash tools/plan_site_mission.sh --stl TEAM.stl --units m
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
python_path=${ROVER_MAPPING_PYTHON:-$root/.venv-mapping/bin/python}
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
unset QT_QPA_PLATFORM_PLUGIN_PATH
cd "$root"
exec "$python_path" -m experiments.handheld_mapping.mission_editor "$@"
