// Unit tests for pTurnLimit's TurnLimitModel. Plain g++, no MOOS.
// Run via tests/run_tests.sh.

#include "TurnLimitModel.h"
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

static int g_checks = 0, g_failed = 0;
#define CHECK(cond, msg) do { g_checks++; if(!(cond)) { g_failed++; \
  cout << "FAIL " << __LINE__ << ": " << msg << endl; } } while(0)

static void testConfig()
{
  TurnLimitModel m;
  CHECK(m.setHeadingTrigger(60), "trigger 60 accepted");
  CHECK(!m.setHeadingTrigger(0) && !m.setHeadingTrigger(200), "trigger out of range refused");
  CHECK(m.setHeadingRelease(20), "release 20 accepted");
  CHECK(!m.setHeadingRelease(60) && !m.setHeadingRelease(-1), "release >= trigger refused");
  CHECK(!m.setHeadingTrigger(20), "trigger <= release refused");
  CHECK(m.setCornerDist(0) && !m.setCornerDist(-5), "corner dist 0 ok, negative refused");
  CHECK(!m.setCornerAngle(0) && m.setCornerAngle(90), "corner angle range");
  CHECK(!m.setHoldSecs(-1) && m.setHoldSecs(0), "hold secs range");
  CHECK(!m.setDesiredStaleSecs(0) && m.setDesiredStaleSecs(2), "stale secs range");
}

static void testTurnAngles()
{
  TurnLimitModel m;
  // 0,0 -> 0,100 (north) -> 100,100 (east) -> 100,0 (south)
  m.applyWptUpdate("polygon=0,0:0,100:100,100:100,0");
  CHECK(m.routeSize() == 4, "route parsed");
  CHECK(fabs(m.turnAt(1, 0, 0) - 90) < 1e-6, "right angle at point 1");
  CHECK(fabs(m.turnAt(2, 0, 0) - 90) < 1e-6, "right angle at point 2");
  CHECK(m.turnAt(3, 0, 0) < 0, "no turn at the end of a one-off route");
  CHECK(fabs(m.turnAt(0, 0, -100) - 0) < 1e-6, "first point: incoming from the boat");
  m.applyWptUpdate("repeat=1000");
  CHECK(fabs(m.turnAt(3, 0, 0) - 90) < 1e-6, "repeat: last point turns to point 0");
  m.applyWptUpdate("loop_start=1");
  // incoming south (100,100 -> 100,0), outgoing to (0,100): bearing 315
  CHECK(fabs(m.turnAt(3, 0, 0) - 135) < 1e-6, "repeat with loop_start=1: " << m.turnAt(3, 0, 0));
  // there-and-back: 180 degrees
  m.applyWptUpdate("polygon=0,0:0,500:0,0 # repeat=0");
  CHECK(fabs(m.turnAt(1, 0, 0) - 180) < 1e-6, "U-turn is 180");
  m.applyWptUpdate("points=pts={0,0:10,10}");
  CHECK(m.routeSize() == 2, "points=pts={...} form parsed");
  m.applyWptUpdate("speed=5 # capture_radius=15");
  CHECK(m.routeSize() == 2, "non-route fields leave the route alone");
}

static void testCornerTrigger()
{
  TurnLimitModel m;
  m.setHoldSecs(0);
  m.applyWptUpdate("polygon=0,0:0,500:500,500");   // 90 deg at point 1
  m.setWptIndex(1);
  double t = 0;
  CHECK(!m.update(t += 1, true, true, 0, 300, false, 0), "200 m before the corner: not limited");
  CHECK(m.update(t += 1, true, true, 0, 420, false, 0), "80 m before a 90 deg corner: limited");
  CHECK(m.reason() == "corner", "reason corner");
  CHECK(!m.update(t += 1, false, true, 0, 420, false, 0), "manual / no mode: never limited");
  // shallow corner below the 60 deg default is ignored
  m.applyWptUpdate("polygon=0,0:0,500:200,900");     // about 27 deg
  m.setWptIndex(1);
  CHECK(!m.update(t += 1, true, true, 0, 450, false, 0), "shallow corner ignored");
  // the last point of a one-off route is not a corner
  m.applyWptUpdate("polygon=0,0:0,500 # repeat=0");
  m.setWptIndex(1);
  CHECK(!m.update(t += 1, true, true, 0, 450, false, 0), "end of route ignored");
  // corner_dist 0 switches corners off
  m.setCornerDist(0);
  m.applyWptUpdate("polygon=0,0:0,500:500,500");
  m.setWptIndex(1);
  CHECK(!m.update(t += 1, true, true, 0, 450, false, 0), "corners off");
}

static void testHeadingTrigger()
{
  TurnLimitModel m;
  m.setHoldSecs(0);
  double t = 100;
  m.setDesiredHeading(90, t);
  CHECK(!m.update(t, true, false, 0, 0, true, 60), "30 deg off: not limited");
  CHECK(m.update(t, true, false, 0, 0, true, 30), "60 deg off: limited");
  CHECK(m.reason() == "heading", "reason heading");
  CHECK(m.update(t, true, false, 0, 0, true, 70), "20 deg off: still limited (hysteresis)");
  CHECK(!m.update(t, true, false, 0, 0, true, 80), "10 deg off: released");
  m.setDesiredHeading(350, t);
  CHECK(!m.update(t, true, false, 0, 0, true, 10), "wraps across north (20 deg)");
  CHECK(m.update(t, true, false, 0, 0, true, 60), "wraps across north (70 deg)");
  // a stale desired heading is not used
  CHECK(!m.update(t + 10, true, false, 0, 0, true, 60), "stale desired heading ignored");
}

static void testHold()
{
  TurnLimitModel m;
  m.setHoldSecs(3);
  double t = 0;
  m.setDesiredHeading(180, t);
  CHECK(m.update(t, true, false, 0, 0, true, 0), "U-turn starts: limited");
  m.setDesiredHeading(0, t + 1);
  CHECK(m.update(t + 1, true, false, 0, 0, true, 0), "turn done: held");
  CHECK(m.reason() == "hold", "reason hold");
  CHECK(m.update(t + 3.5, true, false, 0, 0, true, 0) == false, "released after hold_secs");
}

int main()
{
  testConfig();
  testTurnAngles();
  testCornerTrigger();
  testHeadingTrigger();
  testHold();
  cout << g_checks << " checks, " << g_failed << " failed" << endl;
  return g_failed ? 1 : 0;
}
