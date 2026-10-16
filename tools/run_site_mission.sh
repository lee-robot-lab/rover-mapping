#!/usr/bin/env bash
# ================================================================
# run_site_mission.sh
# 설명: 저장 임무를 합성 재생하거나 명시한 site pose JSONL로 실행한다. 모터 송신 없음.
# 사용법: bash tools/run_site_mission.sh MISSION OUTPUT --scenario stall30
#   또는: bash tools/run_site_mission.sh --input POSES.jsonl MISSION OUTPUT
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
python_path=${ROVER_MAPPING_PYTHON:-$root/.venv-mapping/bin/python}
export PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
cd "$root"
if [[ "${1:-}" == "--input" ]]; then
    if [[ $# -ne 4 ]]; then echo 'usage: --input POSES.jsonl MISSION OUTPUT' >&2; exit 2; fi
    exec "$python_path" -m experiments.handheld_mapping.mission_runtime "$3" "$2" "$4"
fi
exec "$python_path" -m experiments.handheld_mapping.mission_replay "$@"
