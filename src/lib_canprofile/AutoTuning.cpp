//------------------------------------------------------------------
// AutoTuning: autopilot settings from the boat profile. See AutoTuning.h.
//------------------------------------------------------------------

#include "AutoTuning.h"
#include "CanProfile.h"

#include <cstdlib>
#include <sstream>

using namespace std;

namespace canprofile {

// Feed-forward recalibrated from MOOSLog_29_5_2026_____09_33_03 (the table
// that used to live in planchaPID's CalculateSpeed) and the old speed-gain
// schedule.
const char* const DEFAULT_AUTO_SPEED_FF =
  "0:0|3:8|5:14|7:22|8:27|10:35|12:36|15:37|18:41|20:44";
const char* const DEFAULT_AUTO_SPEED_GAINS =
  "2:0.35:0.005|6:0.25:0.004|12:0.16:0.003|1000:0.10:0.002";

static vector<string> splitOn(const string& s, char c)
{
  vector<string> out;
  string item;
  istringstream in(s);
  while(getline(in, item, c))
    out.push_back(item);
  return out;
}

static bool toDouble(const string& s, double& v)
{
  size_t a = s.find_first_not_of(" \t");
  size_t b = s.find_last_not_of(" \t");
  if(a == string::npos)
    return false;
  string t = s.substr(a, b - a + 1);
  char* end = 0;
  v = strtod(t.c_str(), &end);
  return end && *end == '\0';
}

AutoTuning::AutoTuning()
{
  heading_kp        = 0.15;
  heading_ki        = 0.0;
  heading_kd        = 0.26;
  heading_limit     = 20;
  rudder_max        = 20;
  rudder_min        = 8;
  rudder_speed_low  = 5;
  rudder_speed_high = 18;
  thrust_max        = 50;
  i_authority       = 2;
  string err;
  parseSpeedFF(DEFAULT_AUTO_SPEED_FF, speed_ff, err);
  parseGains(DEFAULT_AUTO_SPEED_GAINS, speed_gains, err);
}

bool AutoTuning::parseSpeedFF(const string& s, vector<SpeedPoint>& out, string& err)
{
  return parseSpeedPct(s, "auto_speed_ff", out, err);
}

bool AutoTuning::parseSpeedPct(const string& s, const string& key,
                               vector<SpeedPoint>& out, string& err)
{
  vector<SpeedPoint> pts;
  vector<string> items = splitOn(s, '|');
  for(size_t i=0; i<items.size(); i++) {
    vector<string> f = splitOn(items[i], ':');
    SpeedPoint p;
    if(f.size() != 2 || !toDouble(f[0], p.speed) || !toDouble(f[1], p.thrust)) {
      err = key + " must look like 0:0|5:14|10:35 (speed:percent)";
      return false;
    }
    if(p.thrust < 0 || p.thrust > 100) {
      err = key + ": percent must be 0..100";
      return false;
    }
    if(!pts.empty() && p.speed <= pts.back().speed) {
      err = key + ": speeds must increase";
      return false;
    }
    pts.push_back(p);
  }
  if(pts.size() < 2) {
    err = key + " needs at least two points";
    return false;
  }
  out = pts;
  return true;
}

bool AutoTuning::parseGains(const string& s, vector<GainStep>& out, string& err)
{
  vector<GainStep> steps;
  vector<string> items = splitOn(s, '|');
  for(size_t i=0; i<items.size(); i++) {
    vector<string> f = splitOn(items[i], ':');
    GainStep g;
    if(f.size() != 3 || !toDouble(f[0], g.up_to) || !toDouble(f[1], g.kp) ||
       !toDouble(f[2], g.ki)) {
      err = "auto_speed_gains must look like 2:0.35:0.005|6:0.25:0.004 (up_to:kp:ki)";
      return false;
    }
    if(g.kp < 0 || g.ki < 0) {
      err = "auto_speed_gains: kp and ki must not be negative";
      return false;
    }
    if(!steps.empty() && g.up_to <= steps.back().up_to) {
      err = "auto_speed_gains: speeds must increase";
      return false;
    }
    steps.push_back(g);
  }
  if(steps.empty()) {
    err = "auto_speed_gains needs at least one step";
    return false;
  }
  out = steps;
  return true;
}

bool AutoTuning::configure(const Profile& p, string* err)
{
  string e;
  vector<SpeedPoint> ff;
  vector<GainStep> g;
  if(!parseSpeedFF(p.stringSetting("auto_speed_ff"), ff, e) ||
     !parseGains(p.stringSetting("auto_speed_gains"), g, e)) {
    if(err) *err = e;
    return false;
  }
  speed_ff          = ff;
  speed_gains       = g;
  heading_kp        = p.setting("auto_heading_kp");
  heading_ki        = p.setting("auto_heading_ki");
  heading_kd        = p.setting("auto_heading_kd");
  heading_limit     = p.setting("auto_heading_limit_pct");
  rudder_max        = p.setting("auto_rudder_max_pct");
  rudder_min        = p.setting("auto_rudder_min_pct");
  rudder_speed_low  = p.setting("auto_rudder_speed_low");
  rudder_speed_high = p.setting("auto_rudder_speed_high");
  thrust_max        = p.setting("auto_thrust_max_pct");
  i_authority       = p.setting("auto_speed_i_authority_pct");
  return true;
}

// Linear interpolation, held flat beyond the first and last points
double AutoTuning::interpSpeed(const vector<SpeedPoint>& pts, double v)
{
  if(pts.empty())
    return 0;
  if(v <= pts.front().speed)
    return pts.front().thrust;
  for(size_t i=1; i<pts.size(); i++) {
    if(v <= pts[i].speed) {
      const SpeedPoint& a = pts[i-1];
      const SpeedPoint& b = pts[i];
      double w = (v - a.speed) / (b.speed - a.speed);
      return a.thrust + w * (b.thrust - a.thrust);
    }
  }
  return pts.back().thrust;
}

double AutoTuning::feedForward(double v) const
{
  return interpSpeed(speed_ff, v);
}

// First step whose up_to is above the desired speed; the last step
// covers everything faster.
void AutoTuning::gains(double v, double& kp, double& ki) const
{
  kp = ki = 0;
  for(size_t i=0; i<speed_gains.size(); i++) {
    kp = speed_gains[i].kp;
    ki = speed_gains[i].ki;
    if(v < speed_gains[i].up_to)
      return;
  }
}

double AutoTuning::rudderLimit(double speed) const
{
  if(speed <= rudder_speed_low)
    return rudder_max;
  if(speed >= rudder_speed_high)
    return rudder_min;
  double w = (speed - rudder_speed_low) / (rudder_speed_high - rudder_speed_low);
  return rudder_max + w * (rudder_min - rudder_max);
}

}  // namespace canprofile
