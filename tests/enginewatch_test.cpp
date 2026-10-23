// Unit tests for pEngineWatch's EngineWatchModel. Plain g++, no MOOS.
// Run via tests/run_tests.sh.

#include "EngineWatchModel.h"
#include <iostream>
#include <string>
#include <vector>

using namespace std;

static int g_checks = 0, g_failed = 0;
#define CHECK(cond, msg) do { g_checks++; if(!(cond)) { g_failed++; \
  cout << "FAIL " << __LINE__ << ": " << msg << endl; } } while(0)

typedef EngineWatchModel M;

// An engine reporting at 1 Hz like the Signal K poll / iCANRec.
struct Engine {
  M m;
  double t;
  bool feed;
  Engine() : t(1000), feed(true) {
    m.setEngineName("Port engine");
    m.coolantTemp(80); m.batteryVoltage(13.8); m.alternatorVoltage(14.1); m.fuelLevel(46);
  }
  // run for secs at rpm; returns every change seen
  vector<M::Change> run(double secs, double rpm) {
    vector<M::Change> all;
    for(int i = 0; i < (int)(secs + 0.5); i++) {
      t += 1;
      if(feed) m.rpm(t, rpm);
      vector<M::Change> c = m.update(t);
      all.insert(all.end(), c.begin(), c.end());
    }
    return all;
  }
};

static M::Level level(const M& m, const string& key)
{
  map<string, M::Alert>::const_iterator p = m.active().find(key);
  return p == m.active().end() ? M::NONE : p->second.level;
}

static void testRunState()
{
  Engine e;
  CHECK(e.m.update(e.t).empty() && e.m.runState() == M::UNKNOWN, "no data yet = unknown");
  e.run(1, 4800);
  CHECK(e.m.runState() == M::RUNNING, "4800 rpm = running");
  e.run(1, 299);
  CHECK(e.m.runState() == M::STOPPED, "299 rpm = stopped");
  e.run(1, 300);
  CHECK(e.m.runState() == M::RUNNING, "300 rpm = running");
  e.feed = false;
  e.run(9, 0);
  CHECK(e.m.runState() == M::RUNNING, "9 s without data still running");
  e.run(1, 0);
  CHECK(e.m.runState() == M::UNKNOWN, "10 s without data = unknown");
}

static void testShutOffInMission()
{
  Engine e;
  e.m.setMission(true);
  e.run(5, 4800);
  CHECK(e.m.active().empty(), "running in mission: no alert");
  e.run(2, 0);
  CHECK(level(e.m, "shutOffInMission") == M::NONE, "2 s stopped: not yet (RPM dropout)");
  vector<M::Change> c = e.run(2, 0);
  CHECK(level(e.m, "shutOffInMission") == M::EMERGENCY, "stopped 3 s in mission = emergency");
  CHECK(c.size() == 1 && c[0].key == "shutOffInMission" && c[0].msg == "Port engine SHUT OFF during mission",
        "one change with the message");
  c = e.run(5, 0);
  CHECK(c.empty(), "no repeat while it holds");
  c = e.run(1, 900);
  CHECK(c.size() == 1 && c[0].level == M::NONE && e.m.active().empty(), "restart clears it");

  // not in a mission: shutting the engine off is the operator's choice
  e.m.setMission(false);
  e.run(10, 0);
  CHECK(e.m.active().empty(), "stopped outside a mission: no alert");

  // mission ends while the alert is up
  e.m.setMission(true);
  e.run(4, 0);
  CHECK(level(e.m, "shutOffInMission") == M::EMERGENCY, "raised again");
  e.m.setMission(false);
  e.run(1, 0);
  CHECK(level(e.m, "shutOffInMission") == M::NONE, "mission end clears it");
}

static void testNoData()
{
  Engine e;
  e.m.setMission(true);
  e.feed = false;
  e.run(20, 0);
  CHECK(e.m.active().empty(), "never had data: no noData alert");
  e.feed = true;
  e.run(2, 4800);
  e.feed = false;
  e.run(10, 0);
  CHECK(level(e.m, "noDataInMission") == M::WARN, "data lost 10 s in mission = warn");
  CHECK(level(e.m, "shutOffInMission") == M::NONE, "unknown is not shut off");
  e.feed = true;
  e.run(1, 4800);
  CHECK(level(e.m, "noDataInMission") == M::NONE, "data back clears it");
}

static void testOverheat()
{
  Engine e;
  e.run(1, 4800);
  e.m.coolantTemp(96); e.run(1, 4800);
  CHECK(level(e.m, "overheat") == M::WARN, "96 C = warn");
  e.m.coolantTemp(101); vector<M::Change> c = e.run(1, 4800);
  CHECK(level(e.m, "overheat") == M::ALARM && c.size() == 1 && c[0].msg == "Port engine OVERHEATING: 101 C",
        "101 C = alarm");
  e.m.coolantTemp(94); e.run(1, 4800);
  CHECK(level(e.m, "overheat") == M::WARN, "94 C after alarm = warn (hysteresis)");
  e.m.coolantTemp(93.5); e.run(1, 4800);
  CHECK(level(e.m, "overheat") == M::WARN, "93.5 C still warn");
  e.m.coolantTemp(92); e.run(1, 4800);
  CHECK(level(e.m, "overheat") == M::NONE, "92 C clears");
  e.m.coolantTemp(94); e.run(1, 4800);
  CHECK(level(e.m, "overheat") == M::NONE, "94 C from normal: no alert");

  // held while data is missing
  e.m.coolantTemp(101); e.run(1, 4800);
  e.feed = false; e.run(10, 0);
  CHECK(e.m.runState() == M::UNKNOWN, "data gone");
  e.m.coolantTemp(20); e.run(5, 0);
  CHECK(level(e.m, "overheat") == M::ALARM, "alarm holds while no data");
}

static void testVoltageAndCharging()
{
  Engine e;
  e.m.batteryVoltage(11.9); e.run(1, 4800);
  CHECK(level(e.m, "lowVoltage") == M::WARN, "11.9 V = warn");
  e.m.batteryVoltage(11.4); e.run(1, 4800);
  CHECK(level(e.m, "lowVoltage") == M::ALARM, "11.4 V = alarm");
  e.m.batteryVoltage(12.1); e.run(1, 4800);
  CHECK(level(e.m, "lowVoltage") == M::WARN, "12.1 V after low = warn");
  e.m.batteryVoltage(12.2); e.run(1, 4800);
  CHECK(level(e.m, "lowVoltage") == M::NONE, "12.2 V clears");
  e.m.batteryVoltage(0); e.run(1, 4800);
  CHECK(level(e.m, "lowVoltage") == M::NONE, "0 V = no reading");

  e.m.alternatorVoltage(12.3);
  e.run(29, 4800);
  CHECK(level(e.m, "notCharging") == M::NONE, "29 s not charging: not yet");
  e.run(2, 4800);
  CHECK(level(e.m, "notCharging") == M::WARN, "31 s not charging = warn");
  e.run(1, 0);
  CHECK(level(e.m, "notCharging") == M::NONE, "engine stopped clears it");

  Engine n;   // no alternator reading (MOOS source)
  n.m = M(); n.run(40, 4800);
  CHECK(n.m.active().empty(), "no alternator reading: no alert");
}

static void testFuel()
{
  Engine e;
  e.m.fuelLevel(15); e.run(1, 4800);
  CHECK(level(e.m, "lowFuel") == M::WARN, "15 % = warn");
  e.m.fuelLevel(9); e.run(1, 4800);
  CHECK(level(e.m, "lowFuel") == M::ALARM, "9 % = alarm");
  e.m.fuelLevel(21); e.run(1, 4800);
  CHECK(level(e.m, "lowFuel") == M::WARN, "21 % after low = warn");
  e.m.fuelLevel(22); e.run(1, 4800);
  CHECK(level(e.m, "lowFuel") == M::NONE, "22 % clears");
}

static void testNames()
{
  CHECK(M::levelName(M::NONE) == "clear" && M::levelName(M::EMERGENCY) == "emergency", "level names");
  CHECK(M::runStateName(M::STOPPED) == "stopped", "run state names");
  Engine e;
  e.m.coolantTemp(100); e.m.batteryVoltage(11); e.m.fuelLevel(5);
  vector<M::Change> c = e.run(1, 4800);
  for(size_t i = 0; i < c.size(); i++)
    CHECK(c[i].msg.find(',') == string::npos, "no comma in message: " + c[i].msg);
}

int main()
{
  testRunState();
  testShutOffInMission();
  testNoData();
  testOverheat();
  testVoltageAndCharging();
  testFuel();
  testNames();
  cout << g_checks << " checks, " << g_failed << " failed" << endl;
  return g_failed ? 1 : 0;
}
