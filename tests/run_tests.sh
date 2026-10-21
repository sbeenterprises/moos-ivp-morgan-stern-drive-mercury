#!/bin/bash
#------------------------------------------------------------------
# Builds and runs the lib_canprofile, pCorridor (CorridorModel),
# lib_regiongeom, pTurnLimit (TurnLimitModel) and pColavScale
# (ColavScaleModel) tests with plain g++. No MOOS or CAN hardware needed. Run from anywhere:
#   tests/run_tests.sh
#------------------------------------------------------------------

set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TMPDIR:-/tmp}/canprofile_test.$$"
OUT_CORRIDOR="${TMPDIR:-/tmp}/corridor_test.$$"
OUT_REGION="${TMPDIR:-/tmp}/region_test.$$"
OUT_TURN="${TMPDIR:-/tmp}/turnlimit_test.$$"
OUT_COLAV="${TMPDIR:-/tmp}/colavscale_test.$$"

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

g++ -std=c++11 -Wall -Wextra -O1 \
    -I"$ROOT/src/lib_regiongeom" \
    "$ROOT/src/lib_regiongeom/RegionGeom.cpp" \
    "$ROOT/tests/region_test.cpp" \
    -o "$OUT_REGION"

g++ -std=c++11 -Wall -Wextra -O1 \
    -I"$ROOT/src/pTurnLimit" \
    "$ROOT/src/pTurnLimit/TurnLimitModel.cpp" \
    "$ROOT/tests/turnlimit_test.cpp" \
    -o "$OUT_TURN"

g++ -std=c++11 -Wall -Wextra -O1 \
    -I"$ROOT/src/pColavScale" \
    "$ROOT/src/pColavScale/ColavScaleModel.cpp" \
    "$ROOT/tests/colavscale_test.cpp" \
    -o "$OUT_COLAV"

trap 'rm -f "$OUT" "$OUT_CORRIDOR" "$OUT_REGION" "$OUT_TURN" "$OUT_COLAV"' EXIT
echo "== lib_canprofile =="
"$OUT" "$ROOT/profiles"
echo "== pCorridor CorridorModel =="
"$OUT_CORRIDOR"
echo "== lib_regiongeom =="
"$OUT_REGION"
echo "== pTurnLimit TurnLimitModel =="
"$OUT_TURN"
echo "== pColavScale ColavScaleModel =="
"$OUT_COLAV"
