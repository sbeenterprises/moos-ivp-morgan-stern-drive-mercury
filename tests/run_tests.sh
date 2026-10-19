#!/bin/bash
#------------------------------------------------------------------
# Builds and runs the lib_canprofile and pCorridor (CorridorModel)
# tests with plain g++. No MOOS or CAN hardware needed. Run from anywhere:
#   tests/run_tests.sh
#------------------------------------------------------------------

set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TMPDIR:-/tmp}/canprofile_test.$$"
OUT_CORRIDOR="${TMPDIR:-/tmp}/corridor_test.$$"

g++ -std=c++11 -Wall -Wextra -O1 \
    -I"$ROOT/src/lib_canprofile" \
    "$ROOT/src/lib_canprofile/CanProfile.cpp" \
    "$ROOT/src/lib_canprofile/DriveCore.cpp" \
    "$ROOT/src/lib_canprofile/AutoTuning.cpp" \
    "$ROOT/tests/canprofile_test.cpp" \
    -o "$OUT"

g++ -std=c++11 -Wall -Wextra -O1 \
    -I"$ROOT/src/pCorridor" \
    "$ROOT/src/pCorridor/CorridorModel.cpp" \
    "$ROOT/tests/corridor_test.cpp" \
    -o "$OUT_CORRIDOR"

trap 'rm -f "$OUT" "$OUT_CORRIDOR"' EXIT
echo "== lib_canprofile =="
"$OUT" "$ROOT/profiles"
echo "== pCorridor CorridorModel =="
"$OUT_CORRIDOR"
