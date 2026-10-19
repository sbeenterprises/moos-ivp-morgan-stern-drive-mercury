//------------------------------------------------------------------
// Tests for pCorridor's CorridorModel (cross-track error, zones,
// hysteresis, recover point, inactive handling, spec parsing).
//
// Build and run:   tests/run_tests.sh
// Needs only g++; no MOOS.
//------------------------------------------------------------------

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "CorridorModel.h"

using namespace std;

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond, what)                                              \
  do {                                                                 \
    g_checks++;                                                        \
    if(!(cond)) {                                                      \
      g_failures++;                                                    \
      cout << "FAIL " << __LINE__ << ": " << what << endl;             \
    }                                                                  \
  } while(0)

static bool near(double a, double b, double tol = 1e-6)
{
  return fabs(a - b) <= tol;
}

// L-shaped centreline: (0,0) -> (100,0) -> (100,100). Width 40, half 20.
static CorridorModel makeL()
{
  CorridorModel m;
  vector<double> xs, ys;
  xs.push_back(0);   ys.push_back(0);
  xs.push_back(100); ys.push_back(0);
  xs.push_back(100); ys.push_back(100);
  string err;
  m.setGeometry(40, xs, ys, err);
  return m;
}

static void testGeometryValidation()
{
  CorridorModel m;
  string err;
  vector<double> xs, ys;

  xs.push_back(0); ys.push_back(0);
  CHECK(!m.setGeometry(40, xs, ys, err), "one point rejected");
  CHECK(!m.geometryValid(), "invalid after one point");

  xs.push_back(10); ys.push_back(0);
  CHECK(!m.setGeometry(0, xs, ys, err), "zero width rejected");
  CHECK(!m.setGeometry(-5, xs, ys, err), "negative width rejected");
  CHECK(m.setGeometry(40, xs, ys, err), "two points accepted: " + err);
  CHECK(m.geometryValid(), "valid after two points");

  vector<double> sx(2, 5), sy(2, 5);
  CHECK(!m.setGeometry(40, sx, sy, err), "zero-length centreline rejected");
  CHECK(!m.geometryValid(), "failed set clears the old geometry");
}

static void testSpecParsing()
{
  CorridorModel m;
  string err;

  CHECK(m.setGeometryFromSpec("width=40,pts={0,0:100,0:100,100}", err),
        "good spec: " + err);
  CHECK(m.size() == 3 && near(m.width(), 40), "spec gives 3 points, width 40");

  CHECK(m.setGeometryFromSpec("width=12.5,pts={-6289.3,400.1:-6000,700}", err),
        "negative/decimal coords: " + err);
  CHECK(near(m.width(), 12.5), "decimal width");

  CHECK(!m.setGeometryFromSpec("width=abc,pts={0,0:1,1}", err), "bad width");
  CHECK(!m.setGeometryFromSpec("width=40,pts={0,0:1,x}", err), "bad point");
  CHECK(!m.setGeometryFromSpec("width=40,pts={0,0}", err), "single point");
  CHECK(!m.setGeometryFromSpec("width=40,pts={0,0:1,1", err), "unclosed pts");
  CHECK(!m.setGeometryFromSpec("pts={0,0:1,1}", err), "missing width");
  CHECK(!m.setGeometryFromSpec("", err), "empty spec");
  CHECK(!m.geometryValid(), "invalid after a bad spec");
}

static void testXte()
{
  CorridorModel m = makeL();
  unsigned int seg; double t, px, py, d;

  CHECK(m.closest(50, 10, seg, t, px, py, d), "closest ok");
  CHECK(seg == 0 && near(t, 0.5) && near(d, 10), "mid first segment, 10 m off");
  CHECK(near(px, 50) && near(py, 0), "foot of perpendicular");

  m.closest(50, -7, seg, t, px, py, d);
  CHECK(near(d, 7), "other side gives absolute distance");

  m.closest(110, 50, seg, t, px, py, d);
  CHECK(seg == 1 && near(d, 10), "second segment");

  // Outside the corner: nearest is the vertex (100,0).
  m.closest(110, -10, seg, t, px, py, d);
  CHECK(near(d, sqrt(200.0)) && near(px, 100) && near(py, 0), "vertex distance");

  // Before the start: nearest is the first vertex.
  m.closest(-30, 40, seg, t, px, py, d);
  CHECK(seg == 0 && near(t, 0) && near(d, 50), "before start -> first vertex");

  // Past the end: nearest is the last vertex.
  m.closest(100, 130, seg, t, px, py, d);
  CHECK(seg == 1 && near(t, 1) && near(d, 30), "past end -> last vertex");

  // update() reports the same xte.
  m.update(50, 10, true);
  CHECK(near(m.xte(), 10), "update xte");
}

static void testZonesNoHysteresis()
{
  CorridorModel m = makeL();
  m.setHysteresis(0);
  // half = 20; slow at > 10; recovering at > 20; stop at > 40.
  CHECK(m.rawZone(0)    == CorridorModel::ZONE_IN, "0 in");
  CHECK(m.rawZone(10)   == CorridorModel::ZONE_IN, "10 in (boundary inclusive)");
  CHECK(m.rawZone(10.1) == CorridorModel::ZONE_SLOW, "10.1 slow");
  CHECK(m.rawZone(20)   == CorridorModel::ZONE_SLOW, "20 slow (boundary inclusive)");
  CHECK(m.rawZone(20.1) == CorridorModel::ZONE_RECOVERING, "20.1 recovering");
  CHECK(m.rawZone(40)   == CorridorModel::ZONE_RECOVERING, "40 recovering");
  CHECK(m.rawZone(40.1) == CorridorModel::ZONE_STOP, "40.1 stop");

  CHECK(m.update(50, 5, true)  == CorridorModel::ZONE_IN, "walk: in");
  CHECK(m.update(50, 15, true) == CorridorModel::ZONE_SLOW, "walk: slow");
  CHECK(m.update(50, 30, true) == CorridorModel::ZONE_RECOVERING, "walk: recovering");
  CHECK(m.update(50, 50, true) == CorridorModel::ZONE_STOP, "walk: stop");
  CHECK(m.update(50, 5, true)  == CorridorModel::ZONE_IN, "walk back: in at once with h=0");

  CHECK(CorridorModel::zoneName(CorridorModel::ZONE_RECOVERING) == "recovering", "name");
  CHECK(CorridorModel::zoneName(CorridorModel::ZONE_NONE) == "none", "name none");
}

static void testHysteresis()
{
  CorridorModel m = makeL();   // default hysteresis 2 m
  CHECK(near(m.hysteresis(), 2), "default hysteresis");

  CHECK(m.update(50, 12, true) == CorridorModel::ZONE_SLOW, "12 slow");
  CHECK(m.update(50, 9, true)  == CorridorModel::ZONE_SLOW, "9 stays slow (needs < 8)");
  CHECK(m.update(50, 8.5, true) == CorridorModel::ZONE_SLOW, "8.5 stays slow");
  CHECK(m.update(50, 7.9, true) == CorridorModel::ZONE_IN, "7.9 back in");
  CHECK(m.update(50, 10.5, true) == CorridorModel::ZONE_SLOW, "worse is immediate");

  CHECK(m.update(50, 25, true) == CorridorModel::ZONE_RECOVERING, "25 recovering");
  CHECK(m.update(50, 19, true) == CorridorModel::ZONE_RECOVERING, "19 still recovering");
  CHECK(m.update(50, 17.5, true) == CorridorModel::ZONE_SLOW, "17.5 slow");

  CHECK(m.update(50, 45, true) == CorridorModel::ZONE_STOP, "45 stop");
  CHECK(m.update(50, 39, true) == CorridorModel::ZONE_STOP, "39 still stop");
  CHECK(m.update(50, 37, true) == CorridorModel::ZONE_RECOVERING, "37 recovering");

  // Big jump back while in stop: goes as far as hysteresis allows in one go.
  m.update(50, 45, true);
  CHECK(m.update(50, 3, true) == CorridorModel::ZONE_IN, "stop -> in in one step");
}

static void testInactive()
{
  CorridorModel m = makeL();
  CHECK(m.update(50, 30, false) == CorridorModel::ZONE_NONE, "inactive -> none");
  double rx, ry;
  CHECK(!m.recoverPoint(rx, ry), "no recover point while inactive");

  // Hysteresis memory is dropped on deactivation.
  m.update(50, 30, true);
  m.update(50, 0, false);
  CHECK(m.update(50, 19, true) == CorridorModel::ZONE_SLOW,
        "reactivation classifies from scratch");

  CorridorModel empty;
  CHECK(empty.update(0, 0, true) == CorridorModel::ZONE_NONE, "no geometry -> none");

  // Clearing the geometry while active also gives none.
  m.clearGeometry();
  CHECK(m.update(50, 5, true) == CorridorModel::ZONE_NONE, "cleared -> none");
}

static void testRecoverPoint()
{
  CorridorModel m = makeL();   // recover_lead default 20
  double rx = 0, ry = 0;

  m.update(30, 25, true);
  CHECK(m.recoverPoint(rx, ry), "recover point available");
  CHECK(near(rx, 50) && near(ry, 0), "lead along first segment");

  // Wraps round the corner: closest (90,0), +20 -> 10 m up the second leg.
  m.update(90, -25, true);
  m.recoverPoint(rx, ry);
  CHECK(near(rx, 100) && near(ry, 10), "lead carries round the vertex");

  // Clamped to the end.
  m.update(130, 95, true);
  m.recoverPoint(rx, ry);
  CHECK(near(rx, 100) && near(ry, 100), "clamped to last vertex");

  // Lead zero gives the closest point itself.
  m.setRecoverLead(0);
  m.update(40, -30, true);
  m.recoverPoint(rx, ry);
  CHECK(near(rx, 40) && near(ry, 0), "lead 0 -> closest point");

  CHECK(!m.setRecoverLead(-1), "negative lead rejected");
  CHECK(!m.setSlowFrac(0) && !m.setSlowFrac(1.5), "slow_frac range");
  CHECK(!m.setStopFactor(0.5), "stop_factor < 1 rejected");
  CHECK(!m.setHysteresis(-1), "negative hysteresis rejected");
}

static void testTuning()
{
  CorridorModel m = makeL();
  m.setHysteresis(0);
  m.setSlowFrac(0.8);   // slow above 16
  m.setStopFactor(3);   // stop above 60
  CHECK(m.rawZone(15)  == CorridorModel::ZONE_IN, "slow_frac 0.8: 15 in");
  CHECK(m.rawZone(17)  == CorridorModel::ZONE_SLOW, "slow_frac 0.8: 17 slow");
  CHECK(m.rawZone(55)  == CorridorModel::ZONE_RECOVERING, "stop_factor 3: 55 recovering");
  CHECK(m.rawZone(61)  == CorridorModel::ZONE_STOP, "stop_factor 3: 61 stop");
}

static void testSharpTurns()
{
  string err;
  CorridorModel l = makeL();                     // one 90 deg turn at (100,0)
  CHECK(near(l.turnAngle(1), 90, 1e-6), "L corner turns 90 deg");
  CHECK(near(l.turnAngle(0), 0) && near(l.turnAngle(2), 0), "open ends do not turn");
  CHECK(l.nearSharpTurn(80, 0, 30, 60), "20 m before the corner: slow");
  CHECK(!l.nearSharpTurn(50, 0, 30, 60), "50 m before the corner: not yet");
  CHECK(!l.nearSharpTurn(80, 0, 30, 100), "90 deg is not sharp at a 100 deg limit");

  // there and back: approach east to the route start, route back west
  CorridorModel u;
  vector<double> xs, ys;
  xs.push_back(0);   ys.push_back(0);
  xs.push_back(200); ys.push_back(0);
  xs.push_back(0);   ys.push_back(1);
  CHECK(u.setGeometry(60, xs, ys, err), "U-turn geometry: " + err);
  CHECK(u.turnAngle(1) > 170, "U-turn is close to 180 deg");
  CHECK(u.nearSharpTurn(170, 0, 40, 60), "slow 30 m before a U-turn");

  // closed loop: the start/end joint is a turn too
  CorridorModel c;
  vector<double> cx, cy;
  cx.push_back(0);   cy.push_back(0);
  cx.push_back(100); cy.push_back(0);
  cx.push_back(100); cy.push_back(100);
  cx.push_back(0);   cy.push_back(0.5);
  CHECK(c.setGeometry(40, cx, cy, err), "closed loop geometry: " + err);
  CHECK(c.turnAngle(0) > 100, "closed-loop joint turns sharply");
  CHECK(c.nearSharpTurn(5, 5, 20, 60), "near the loop joint: slow");

  // straight line never slows
  CorridorModel s;
  vector<double> sx, sy;
  sx.push_back(0); sy.push_back(0);
  sx.push_back(100); sy.push_back(0);
  sx.push_back(200); sy.push_back(0);
  CHECK(s.setGeometry(40, sx, sy, err), "straight geometry: " + err);
  CHECK(near(s.turnAngle(1), 0, 1e-6) && !s.nearSharpTurn(100, 0, 50, 10), "no turn on a straight line");
  s.clearGeometry();
  CHECK(!s.nearSharpTurn(100, 0, 50, 10), "no geometry, no turn");
}

int main()
{
  testGeometryValidation();
  testSpecParsing();
  testXte();
  testZonesNoHysteresis();
  testHysteresis();
  testInactive();
  testRecoverPoint();
  testTuning();
  testSharpTurns();

  cout << g_checks << " checks, " << g_failures << " failed" << endl;
  return g_failures ? 1 : 0;
}
