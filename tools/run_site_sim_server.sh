#!/usr/bin/env bash
# ================================================================
# run_site_sim_server.sh
# 설명: 검증된 임무와 코드를 서버 전용 run에 올리고 GPU1 실행·관찰 tunnel을 관리한다.
# 사용법: bash tools/run_site_sim_server.sh upload LABEL MISSION_FOLDER
#   bash tools/run_site_sim_server.sh start LABEL
#   bash tools/run_site_sim_server.sh tunnel LABEL 8765
# ================================================================
set -euo pipefail
cd "$(dirname "$0")/.."
exec "${ROVER_MAPPING_PYTHON:-$PWD/.venv-mapping/bin/python}" -m experiments.site_sim.server "$@"
