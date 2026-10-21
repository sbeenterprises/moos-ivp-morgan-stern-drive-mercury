// Unit tests for pColavScale's ColavScaleModel. Plain g++, no MOOS.
// Run via tests/run_tests.sh.

#include "ColavScaleModel.h"
#include <cmath>
#include <iostream>

using namespace std;

static int g_checks = 0, g_failed = 0;
#define CHECK(cond, msg) do { g_checks++; if(!(cond)) { g_failed++; \
  cout << "FAIL " << __LINE__ << ": " << msg << endl; } } while(0)

static const double KN = 0.514444;   // m/s per knot

static void testConfig()
{
  ColavScaleModel m;
  CHECK(!m.setOuterSecs(0) && m.setOuterSecs(45), "outer secs range");
  CHECK(!m.setInnerSecs(-1) && m.setInnerSecs(20), "inner secs range");
  CHECK(!m.setOuterMin(0) && m.setOuterMin(100), "outer min range");
  CHECK(!m.setInnerMax(0) && m.setInnerMax(500), "inner max range");
  CHECK(!m.setChangeFrac(-0.1) && m.setChangeFrac(0.2), "change frac range");
}

static void testEncounters()
{
  ColavScaleModel m;   // 60 s / 30 s, mins 150 / 75, maxes 1400 / 700
  // head-on: us 5 kn north, contact 10 kn south, 2 km ahead
  ColavDist d = m.distances(0, 0, 0, 5 * KN, 0, 2000, 180, 10 * KN);
  CHECK(fabs(d.rel_speed - 15 * KN) < 1e-6, "head-on closing 15 kn");
  CHECK(fabs(d.outer - 60 * 15 * KN) < 1e-6, "head-on outer 463 m, got " << d.outer);
  CHECK(fabs(d.inner - 30 * 15 * KN) < 1e-6, "head-on inner 232 m, got " << d.inner);
  // fast head-on hits the maximums
  d = m.distances(0, 0, 0, 20 * KN, 0, 3000, 180, 30 * KN);
  CHECK(d.outer == 1400 && d.inner == 700, "50 kn closing capped at 1400 / 700");
  // overtaking a slower boat: 5 kn vs 3 kn same way -> 2 kn -> minimums
  d = m.distances(0, 0, 0, 5 * KN, 0, 500, 0, 3 * KN);
  CHECK(d.outer == 150 && d.inner == 75, "slow overtaking: minimums");
  // being overtaken by a faster boat from astern still counts as closing
  d = m.distances(0, 0, 0, 5 * KN, 0, -800, 0, 15 * KN);
  CHECK(fabs(d.outer - 60 * 10 * KN) < 1e-6, "overtaken at 10 kn relative: " << d.outer);
  // crossing from starboard, collision course: full relative speed used
  d = m.distances(0, 0, 0, 10 * KN, 1000, 1000, 270, 10 * KN);
  CHECK(fabs(d.rel_speed - 10 * KN * sqrt(2.0)) < 1e-6, "crossing uses |relative velocity|");
  // opening range -> minimums
  d = m.distances(0, 0, 0, 10 * KN, 0, -500, 180, 10 * KN);
  CHECK(d.rel_speed == 0 && d.outer == 150 && d.inner == 75, "opening: minimums");
  // stationary contact ahead, us 10 kn
  d = m.distances(0, 0, 0, 10 * KN, 0, 1000, 0, 0);
  CHECK(fabs(d.outer - 60 * 10 * KN) < 1e-6, "stationary target ahead");
  // we are stopped, contact passes far off: still its own speed if closing
  d = m.distances(0, 0, 0, 0, 0, 1000, 180, 6 * KN);
  CHECK(fabs(d.outer - 60 * 6 * KN) < 1e-6, "we are stopped, contact closing");
}

static void testInnerNeverAboveOuter()
{
  ColavScaleModel m;
  m.setOuterMax(300);
  m.setInnerMin(400);   // misconfigured: inner min above outer max
  ColavDist d = m.distances(0, 0, 0, 20 * KN, 0, 3000, 180, 30 * KN);
  CHECK(d.inner <= d.outer, "inner never above outer: " << d.inner << " vs " << d.outer);
}

static void testChanged()
{
  ColavScaleModel m;   // 10 % and 20 m
  ColavDist a, b;
  a.outer = 400; a.inner = 200;
  b.outer = 430; b.inner = 200;
  CHECK(!m.changed(a, b), "30 m on 400 (7.5 %): no re-post");
  b.outer = 450;
  CHECK(m.changed(a, b), "50 m on 400: re-post");
  a.outer = 150; a.inner = 75; b.outer = 165; b.inner = 75;
  CHECK(!m.changed(a, b), "15 m: below the 20 m floor");
}

int main()
{
  testConfig();
  testEncounters();
  testInnerNeverAboveOuter();
  testChanged();
  cout << g_checks << " checks, " << g_failed << " failed" << endl;
  return g_failed ? 1 : 0;
}
