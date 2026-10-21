// Unit tests for pNavGuard's NavGuardModel. Plain g++, no MOOS.
// Run via tests/run_tests.sh.

#include "NavGuardModel.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <string>

using namespace std;

static int g_checks = 0, g_failed = 0;
#define CHECK(cond, msg) do { g_checks++; if(!(cond)) { g_failed++; \
  cout << "FAIL " << __LINE__ << ": " << msg << endl; } } while(0)

// A simulated boat feeding the model at 10 Hz like iDivisorNMEA (RMC + HDT).
struct Boat {
  NavGuardModel& m;
  double t, x, y, spd, trk, rate, accel;   // m/s, deg, deg/s, m/s2
  double hdg_off, sog_override, cog_override, hdg_override;
  bool feed_pos, valid;
  Boat(NavGuardModel& mm) : m(mm), t(0), x(0), y(0), spd(3), trk(0), rate(0), accel(0),
    hdg_off(0), sog_override(-1), cog_override(-1), hdg_override(-1),
    feed_pos(true), valid(true) {}
  // run for secs; returns the worst state seen
  NavGuardModel::State run(double secs) {
    NavGuardModel::State worst = NavGuardModel::OK;
    for(int i = 0; i < (int)(secs * 10 + 0.5); i++) {
      t += 0.1;
      trk = fmod(trk + rate * 0.1 + 360, 360);
      spd = max(0.0, spd + accel * 0.1);
      x += spd * 0.1 * sin(trk * M_PI / 180);
      y += spd * 0.1 * cos(trk * M_PI / 180);
      if(feed_pos)
        m.position(t, 29.7 + y / 6378137.0 * 180 / M_PI,
                   -91.2 + x / (6378137.0 * cos(29.7 * M_PI / 180)) * 180 / M_PI);
      m.gpsValid(t, valid);
      // 0.1 kn / 0.1 deg resolution like the NMEA fields
      m.sog(t, sog_override >= 0 ? sog_override : floor(spd / 0.0514444 + 0.5) * 0.0514444);
      m.cog(t, cog_override >= 0 ? cog_override : floor(trk * 10 + 0.5) / 10);
      m.heading(t, hdg_override >= 0 ? hdg_override : fmod(floor((trk + hdg_off) * 10 + 0.5) / 10 + 360, 360));
      if(i % 4 == 0) {          // 4 Hz AppTick
        NavGuardModel::State s = m.update(t);
        if(s > worst) worst = s;
      }
    }
    return worst;
  }
};

static void testConfig()
{
  NavGuardModel m;
  CHECK(m.setHeadingTol(30) && !m.setHeadingTol(5) && !m.setHeadingTol(200), "heading tol range");
  CHECK(m.setMinSpeed(1) && !m.setMinSpeed(0), "min speed range");
  CHECK(m.setJumpSpeed(25) && !m.setJumpSpeed(1), "jump speed range");
  CHECK(m.setGpsFlagSecs(0) && !m.setGpsFlagSecs(-1), "gps flag secs range");
  CHECK(!m.setFollowSecs(1) && m.setFollowSecs(30), "follow secs range");
  CHECK(NavGuardModel::angleDiff(350, 10) == 20 && NavGuardModel::angleDiff(10, 190) == 180, "angleDiff");
}

static void testGoodRun()
{
  NavGuardModel m; Boat b(m);
  CHECK(m.update(0) == NavGuardModel::FAULT, "no position at start = fault");
  b.run(15);
  CHECK(m.state() == NavGuardModel::OK, "clean data clears the start-up fault");
  CHECK(b.run(60) == NavGuardModel::OK, "straight run, steady SOG: never warns");
  CHECK(fabs(m.gpsSpeed() - 3) < 0.1 && NavGuardModel::angleDiff(m.gpsTrack(), 0) < 2, "track/speed from positions");
  b.rate = 6;
  CHECK(b.run(60) == NavGuardModel::OK, "turning circles: never warns");
  b.rate = 0; b.accel = -0.3; b.run(9); b.accel = 0; b.spd = 0.3; b.hdg_off = 120;
  b.run(30);
  CHECK(m.state() == NavGuardModel::OK, "drifting slowly with any heading: not checked");
}

static void testHeading()
{
  NavGuardModel m; Boat b(m);
  b.run(15);
  b.hdg_off = 130;
  b.run(3);
  CHECK(m.state() == NavGuardModel::WARN, "heading 130 off: warn first");
  CHECK(m.reason().find("heading") != string::npos, "reason names the heading");
  b.run(9);
  CHECK(m.state() == NavGuardModel::FAULT, "heading 130 off for 10 s: fault");
  b.hdg_off = 0;
  b.run(8);
  CHECK(m.state() == NavGuardModel::FAULT, "fault holds while clean < clear_secs");
  b.run(5);
  CHECK(m.state() == NavGuardModel::OK, "fault clears after clear_secs clean");

  NavGuardModel m2; Boat c(m2);
  c.run(15);
  c.hdg_off = 25;
  CHECK(c.run(60) == NavGuardModel::OK, "25 deg crab (current / leeway) is fine");
}

static void testSpeed()
{
  NavGuardModel m; Boat b(m);
  b.run(15);
  b.sog_override = 6;     // reports 6 m/s, really 3
  b.run(5);
  CHECK(m.state() == NavGuardModel::WARN, "speed mismatch: warn");
  b.run(5);
  CHECK(m.state() == NavGuardModel::FAULT && m.reason().find("speed") != string::npos,
        "speed mismatch 8 s: fault");

  NavGuardModel m2; Boat c(m2);
  c.spd = 2; c.run(15);
  c.sog_override = 2;     // value stops updating ...
  c.accel = 0.2;          // ... while the boat speeds up
  c.run(10);
  CHECK(m2.state() == NavGuardModel::FAULT && m2.reason().find("stuck") != string::npos,
        "SOG frozen while the boat accelerates: fault");
}

static void testFrozen()
{
  NavGuardModel m; Boat b(m);
  b.run(15);
  b.hdg_override = 0; b.cog_override = 0;
  CHECK(b.run(15) == NavGuardModel::OK, "constant heading/course on a straight run is fine");
  b.rate = 5;             // turns, values stay put
  b.run(12);
  CHECK(m.state() == NavGuardModel::FAULT, "heading/course frozen in a turn: fault");
}

static void testStaleAndJump()
{
  NavGuardModel m; Boat b(m);
  b.run(15);
  b.feed_pos = false;
  b.run(4);
  CHECK(m.state() == NavGuardModel::FAULT && m.reason().find("position") != string::npos,
        "positions stop: fault");

  NavGuardModel m2; Boat c(m2);
  c.run(15);
  c.x += 200;             // 200 m in 0.1 s
  c.run(0.4);
  CHECK(m2.state() == NavGuardModel::FAULT && m2.reason().find("jump") != string::npos,
        "position jump: immediate fault");
  c.run(12);
  CHECK(m2.state() == NavGuardModel::OK, "jump fault clears afterwards");

  NavGuardModel m3; Boat d(m3);
  d.run(15);
  d.valid = false;
  d.run(1.5);
  CHECK(m3.state() == NavGuardModel::WARN, "RMC V: warn first");
  d.run(1.5);
  CHECK(m3.state() == NavGuardModel::FAULT && m3.reason().find("no fix") != string::npos,
        "RMC V for 2 s: fault");
}

static void testFollow()
{
  NavGuardModel m; Boat b(m);
  b.run(15);
  m.setAutonomy(true);
  for(int i = 0; i < 300; i++) { m.desiredHeading(b.t, 90); b.run(0.1); }
  CHECK(m.state() == NavGuardModel::FAULT && m.reason().find("following") != string::npos,
        "autonomy: track 90 off a steady course for 20 s: fault");

  NavGuardModel m2; Boat c(m2);
  c.run(15);
  m2.setAutonomy(true);
  c.rate = 4;             // turning toward 180 at 4 deg/s
  NavGuardModel::State worst = NavGuardModel::OK;
  for(int i = 0; i < 400 && NavGuardModel::angleDiff(c.trk, 180) > 5; i++) {
    m2.desiredHeading(c.t, 180);
    NavGuardModel::State s = c.run(0.1);
    if(s > worst) worst = s;
  }
  CHECK(worst != NavGuardModel::FAULT, "a long turn toward the course is not a fault");

  NavGuardModel m3; Boat d(m3);
  d.run(15);
  m3.setAutonomy(false);
  for(int i = 0; i < 300; i++) { m3.desiredHeading(d.t, 90); d.run(0.1); }
  CHECK(m3.state() == NavGuardModel::OK, "follow check off outside autonomy");
}

int main()
{
  testConfig();
  testGoodRun();
  testHeading();
  testSpeed();
  testFrozen();
  testStaleAndJump();
  testFollow();
  cout << g_checks << " checks, " << g_failed << " failed" << endl;
  return g_failed ? 1 : 0;
}
