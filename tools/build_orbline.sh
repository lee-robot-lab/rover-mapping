#!/usr/bin/env bash
# ================================================================
# build_orbline.sh
# 설명: 노트북 native 환경에서 별도 ORB-LINE 비교 실행기를 빌드한다.
# 사용법: bash tools/build_orbline.sh [PANGOLIN_PREFIX]
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
prefix=${1:-$root/outputs/handheld_mapping/native_dependencies/pangolin}
test -f "$root/external/ORB-LINE-SLAM/include/System.h"
cmake -S "$root/experiments/handheld_mapping/orbline" -B "$root/outputs/handheld_mapping/orbline_build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$prefix" -DPangolin_DIR="$prefix/lib/cmake/Pangolin"
cmake --build "$root/outputs/handheld_mapping/orbline_build" --target rover_orbline --parallel 1
