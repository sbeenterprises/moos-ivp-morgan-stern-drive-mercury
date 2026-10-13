#!/bin/bash
#------------------------------------------------------------------
# Builds and runs the lib_canprofile tests with plain g++.
# No MOOS or CAN hardware needed. Run from anywhere:
#   tests/run_tests.sh
#------------------------------------------------------------------

set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${TMPDIR:-/tmp}/canprofile_test.$$"

g++ -std=c++11 -Wall -Wextra -O1 \
    -I"$ROOT/src/lib_canprofile" \
    "$ROOT/src/lib_canprofile/CanProfile.cpp" \
    "$ROOT/src/lib_canprofile/DriveCore.cpp" \
    "$ROOT/tests/legacy_mercury_frames.cpp" \
    "$ROOT/tests/canprofile_test.cpp" \
    -o "$OUT"

trap 'rm -f "$OUT"' EXIT
"$OUT" "$ROOT/profiles"
