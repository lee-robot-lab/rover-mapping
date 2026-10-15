#!/usr/bin/env bash
# ================================================================
# build_mapping_baseline.sh
# 설명: 기존 native ORB-SLAM3 공유 라이브러리로 노트북 비교 실행기를 빌드한다.
# 사용법: bash tools/build_mapping_baseline.sh /absolute/path/to/ORB_SLAM3
# ================================================================
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
orb_root=${1:?기존 native ORB_SLAM3 소스·라이브러리 경로가 필요합니다}
prefix="$root/outputs/handheld_mapping/native_dependencies/pangolin"
build="$root/outputs/handheld_mapping/orb3_comparison_build"
if ! nm -D --defined-only "$orb_root/lib/libORB_SLAM3.so" | awk '$3 == "orbslam3_has_stereo_recovery_wait" {wait=1} $3 == "orbslam3_has_initial_stereo_relocalization" {initial=1} $3 == "orbslam3_has_safe_presave_observations" {save=1} END {exit !(wait && initial && save)}'; then
  echo 'native mapping patch가 없거나 구버전입니다. 최신 stereo_recovery_wait 및 presave_observation_safety patch 적용 후 core를 재빌드하세요.' >&2
  exit 1
fi
cmake -S "$root/experiments/handheld_mapping/orbline/baseline" -B "$build" \
  -DORB3_ROOT="$orb_root" -DCMAKE_PREFIX_PATH="$prefix" -DPangolin_DIR="$prefix/lib/cmake/Pangolin"
cmake --build "$build" --parallel 1
mkdir -p "$build/runtime"
for library in "$orb_root/lib/libORB_SLAM3.so" "$orb_root/Thirdparty/DBoW2/lib/libDBoW2.so" \
               "$orb_root/Thirdparty/g2o/lib/libg2o.so"; do
  temporary=$(mktemp "$build/runtime/.native-library.XXXXXX")
  cp "$library" "$temporary"
  mv -f "$temporary" "$build/runtime/$(basename "$library")"
done
