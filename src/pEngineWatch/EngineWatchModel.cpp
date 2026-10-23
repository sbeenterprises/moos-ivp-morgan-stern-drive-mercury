/*****************************************************************/
/*    FILE: EngineWatchModel.cpp                                 */
/*****************************************************************/

#include <cstdio>
#include "EngineWatchModel.h"

using namespace std;

const double EngineWatchModel::kRunningRpm = 300;
const double EngineWatchModel::kStaleSecs = 10;
const double EngineWatchModel::kOffHoldSecs = 3;
const double EngineWatchModel::kChargeHoldSecs = 30;

static string fmt(const char* f, double v)
{
  char buf[64];
  snprintf(buf, sizeof(buf), f, v);
  return buf;
}

EngineWatchModel::EngineWatchModel()
{
  m_name = "Engine";
  m_mission = false;
  m_rpm = 0;
  m_last_rpm_t = -1;
  m_temp = m_batt = m_alt = m_fuel = 0;
  m_has_temp = m_has_batt = m_has_alt = m_has_fuel = false;
  m_state = UNKNOWN;
}

void EngineWatchModel::rpm(double now, double v)
{
  m_rpm = v;
  m_last_rpm_t = now;
}

string EngineWatchModel::runStateName(RunState s)
{
  if(s == RUNNING) return "running";
  if(s == STOPPED) return "stopped";
  return "unknown";
}

string EngineWatchModel::levelName(Level l)
{
  if(l == WARN)      return "warn";
  if(l == ALARM)     return "alarm";
  if(l == EMERGENCY) return "emergency";
  return "clear";
}

EngineWatchModel::Level EngineWatchModel::prev(const string& key) const
{
  map<string, Alert>::const_iterator p = m_active.find(key);
  return p == m_active.end() ? NONE : p->second.level;
}

// true once cond has held continuously for secs
bool EngineWatchModel::held(const string& key, bool cond, double secs, double now)
{
  if(!cond) {
    m_since.erase(key);
    return false;
  }
  if(m_since.find(key) == m_since.end())
    m_since[key] = now;
  return now - m_since[key] >= secs;
}

// Records the alert; reports it only when its level changes (the message
// carries live numbers and would otherwise change on every update).
void EngineWatchModel::set(vector<Change>& out, const string& key, Level level, const string& msg)
{
  Level was = prev(key);
  if(level == NONE) {
    if(was != NONE) {
      m_active.erase(key);
      Change c = {key, NONE, ""};
      out.push_back(c);
    }
    return;
  }
  Alert a = {level, msg};
  m_active[key] = a;
  if(level != was) {
    Change c = {key, level, msg};
    out.push_back(c);
  }
}

vector<EngineWatchModel::Change> EngineWatchModel::update(double now)
{
  vector<Change> out;
  bool fresh = m_last_rpm_t >= 0 && now - m_last_rpm_t < kStaleSecs;
  m_state = !fresh ? UNKNOWN : (m_rpm >= kRunningRpm ? RUNNING : STOPPED);

  // ---- mission
  set(out, "shutOffInMission",
      held("off", m_mission && m_state == STOPPED, kOffHoldSecs, now) ? EMERGENCY : NONE,
      m_name + " SHUT OFF during mission");
  set(out, "noDataInMission",
      (m_mission && m_state == UNKNOWN && m_last_rpm_t >= 0) ? WARN : NONE,
      m_name + ": no engine data for " + fmt("%.0f", now - m_last_rpm_t) + " s during mission");

  if(!fresh)
    return out;   // threshold alerts hold until data returns

  // ---- temperature (clears 2 C below the warn limit)
  if(m_has_temp) {
    double t = m_temp;
    Level l = t >= 100 ? ALARM : (t >= 95 || (prev("overheat") && t >= 93)) ? WARN : NONE;
    set(out, "overheat", l, l == ALARM ? m_name + " OVERHEATING: " + fmt("%.0f", t) + " C"
                                       : m_name + " temperature high: " + fmt("%.0f", t) + " C");
  }

  // ---- charging
  if(m_has_batt) {
    double v = m_batt;
    Level l = (v > 0 && v < 11.5) ? ALARM
            : ((v > 0 && v < 12.0) || (prev("lowVoltage") && v > 0 && v < 12.2)) ? WARN : NONE;
    set(out, "lowVoltage", l, "House battery low: " + fmt("%.2f", v) + " V");
  }
  set(out, "notCharging",
      held("charge", m_state == RUNNING && m_has_alt && m_alt < 12.8, kChargeHoldSecs, now) ? WARN : NONE,
      m_name + " alternator not charging: " + fmt("%.2f", m_alt) + " V");

  // ---- fuel
  if(m_has_fuel) {
    double f = m_fuel;
    Level l = f < 10 ? ALARM : (f < 20 || (prev("lowFuel") && f < 22)) ? WARN : NONE;
    set(out, "lowFuel", l, "Fuel low: " + fmt("%.0f", f) + " %");
  }
  return out;
}
