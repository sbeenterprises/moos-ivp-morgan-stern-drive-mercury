/*****************************************************************/
/*    FILE: EngineWatchModel.h                                   */
/*                                                               */
/*  Engine run state and engine alerts. Plain C++ (no MOOS),     */
/*  unit tested in tests/enginewatch_test.cpp. Same rules as the */
/*  SS UI's EngineMonitor (engine-monitor.ts), so the boat and   */
/*  the UI agree:                                                */
/*   run state  running (rpm >= 300), stopped, unknown (no rpm   */
/*              reading for 10 s)                                */
/*   shutOffInMission  emergency: stopped 3 s while helm DRIVE   */
/*   noDataInMission   warn: unknown while DRIVE, after data     */
/*   overheat    warn >= 95 C, alarm >= 100 C, clears < 93 C     */
/*   lowVoltage  warn < 12.0 V, alarm < 11.5 V, clears >= 12.2 V */
/*   notCharging warn: running, alternator < 12.8 V for 30 s     */
/*   lowFuel     warn < 20 %, alarm < 10 %, clears >= 22 %       */
/*  Threshold alerts are only re-evaluated while readings are    */
/*  fresh; they hold their level while the data is missing.      */
/*****************************************************************/

#ifndef ENGINE_WATCH_MODEL_HEADER
#define ENGINE_WATCH_MODEL_HEADER

#include <map>
#include <string>
#include <vector>

class EngineWatchModel
{
 public:
  enum Level { NONE = 0, WARN = 1, ALARM = 2, EMERGENCY = 3 };
  enum RunState { UNKNOWN = 0, STOPPED = 1, RUNNING = 2 };

  struct Alert { Level level; std::string msg; };
  struct Change { std::string key; Level level; std::string msg; };  // NONE = cleared

  EngineWatchModel();

  void setEngineName(const std::string& name) { m_name = name; }  // "Port engine"
  std::string engineName() const { return m_name; }

  // ---- inputs (now = seconds, any epoch, increasing)
  void rpm(double now, double v);           // a reading: also marks the engine as reporting
  void coolantTemp(double v)       { m_temp = v; m_has_temp = true; }
  void batteryVoltage(double v)    { m_batt = v; m_has_batt = true; }
  void alternatorVoltage(double v) { m_alt = v;  m_has_alt = true; }
  void fuelLevel(double v)         { m_fuel = v; m_has_fuel = true; }
  void setMission(bool on)         { m_mission = on; }

  // ---- evaluation: returns the alerts whose level changed
  std::vector<Change> update(double now);

  RunState runState() const { return m_state; }
  bool mission() const { return m_mission; }
  double lastRpm() const { return m_rpm; }
  double lastTemp() const { return m_has_temp ? m_temp : -1000; }
  bool hasTemp() const { return m_has_temp; }
  double secsSinceData(double now) const { return m_last_rpm_t < 0 ? -1 : now - m_last_rpm_t; }
  const std::map<std::string, Alert>& active() const { return m_active; }

  static std::string runStateName(RunState s);
  static std::string levelName(Level l);     // warn | alarm | emergency | clear

  static const double kRunningRpm;   // 300
  static const double kStaleSecs;    // 10
  static const double kOffHoldSecs;  // 3
  static const double kChargeHoldSecs; // 30

 private:
  void set(std::vector<Change>& out, const std::string& key, Level level, const std::string& msg);
  bool held(const std::string& key, bool cond, double secs, double now);
  Level prev(const std::string& key) const;

  std::string m_name;
  bool   m_mission;
  double m_rpm, m_last_rpm_t;
  double m_temp, m_batt, m_alt, m_fuel;
  bool   m_has_temp, m_has_batt, m_has_alt, m_has_fuel;
  RunState m_state;
  std::map<std::string, Alert>  m_active;
  std::map<std::string, double> m_since;   // when a held condition began
};

#endif
