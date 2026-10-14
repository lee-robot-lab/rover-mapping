#!/usr/bin/env bash
# ================================================================
# terrain_mapping.sh
# 설명: 노트북 매핑 환경에서 지형 정보·costmap 기록 재생 CLI를 실행한다.
# 사용법: bash tools/terrain_mapping.sh --demo --output OUTPUT
# ================================================================
set -euo pipefail
TERRAIN_TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TERRAIN_TASK_ROOT"
export PYTHONNOUSERSITE=1
exec "$TERRAIN_TASK_ROOT/.venv-mapping/bin/python" -m experiments.handheld_mapping.terrain_replay "$@"
