/*****************************************************************/
/*    FILE: navguard_replay.cpp                                  */
/*                                                               */
/*  Runs NavGuardModel over a recorded .alog, the way pNavGuard  */
/*  would have seen it, and prints every state change. No MOOS   */
/*  needed:                                                      */
/*    navguard_replay [options] file.alog                        */
/*  Nav comes from the raw NMEA in MSG_UDP ($--RMC, $--HDT, as   */
/*  iDivisorNMEA parses it) when the log has it, otherwise from  */
/*  NAV_LAT/NAV_LONG/NAV_SPEED/NAV_COG/NAV_HEADING. Autonomy =   */
/*  IVPHELM_ALLSTOP is "clear" (the helm is driving).            */
/*****************************************************************/

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "NavGuardModel.h"

using namespace std;

static const double kKnot = 0.514444;

static vector<string> split(const string& s, char c)
{
  vector<string> out; string cur;
  for(size_t i = 0; i < s.size(); i++) {
    if(s[i] == c) { out.push_back(cur); cur.clear(); }
    else cur += s[i];
  }
  out.push_back(cur);
  return out;
}

// ddmm.mmmm + hemisphere -> degrees
static bool nmeaDeg(const string& v, const string& h, double& out)
{
  if(v.empty() || h.empty()) return false;
  double raw = atof(v.c_str());
  double deg = floor(raw / 100);
  out = deg + (raw - deg * 100) / 60;
  if(h == "S" || h == "W") out = -out;
  return true;
}

static void usage()
{
  cout << "Usage: navguard_replay [options] file.alog\n"
          "  --speed_units=knots|mps  NAV_SPEED units when the log has no NMEA (default knots)\n"
          "  --nav=nmea|vars          force the nav source (default: nmea if present)\n"
          "  --verbose                also print every check's 1 Hz detail\n"
          "  --<param>=<value>        any pNavGuard model parameter in m/s / deg / s:\n"
          "     min_speed heading_tol heading_secs speed_tol speed_secs freeze_secs\n"
          "     heading_freeze_secs stale_secs jump_speed jump_min_dist gps_flag_secs\n"
          "     follow_tol follow_secs clear_secs window_secs\n";
  exit(1);
}

int main(int argc, char* argv[])
{
  NavGuardModel m;
  string file, nav = "auto";
  double spd_scale = kKnot;
  bool verbose = false;

  for(int i = 1; i < argc; i++) {
    string a = argv[i];
    if(a == "-h" || a == "--help") usage();
    if(a == "--verbose") { verbose = true; continue; }
    if(a.compare(0, 2, "--") == 0 && a.find('=') != string::npos) {
      string k = a.substr(2, a.find('=') - 2);
      string v = a.substr(a.find('=') + 1);
      double d = atof(v.c_str());
      bool ok = true;
      if(k == "speed_units") spd_scale = (v == "mps") ? 1.0 : kKnot;
      else if(k == "nav") nav = v;
      else if(k == "min_speed") ok = m.setMinSpeed(d);
      else if(k == "heading_tol") ok = m.setHeadingTol(d);
      else if(k == "heading_secs") ok = m.setHeadingSecs(d);
      else if(k == "speed_tol") ok = m.setSpeedTol(d);
      else if(k == "speed_secs") ok = m.setSpeedSecs(d);
      else if(k == "freeze_secs") ok = m.setFreezeSecs(d);
      else if(k == "heading_freeze_secs") ok = m.setHeadingFreezeSecs(d);
      else if(k == "stale_secs") ok = m.setStaleSecs(d);
      else if(k == "jump_speed") ok = m.setJumpSpeed(d);
      else if(k == "jump_min_dist") ok = m.setJumpMinDist(d);
      else if(k == "gps_flag_secs") ok = m.setGpsFlagSecs(d);
      else if(k == "follow_tol") ok = m.setFollowTol(d);
      else if(k == "follow_secs") ok = m.setFollowSecs(d);
      else if(k == "clear_secs") ok = m.setClearSecs(d);
      else if(k == "window_secs") ok = m.setWindowSecs(d);
      else { cerr << "unknown option " << a << endl; usage(); }
      if(!ok) { cerr << "value out of range: " << a << endl; return 1; }
      continue;
    }
    file = a;
  }
  if(file.empty()) usage();

  if(nav == "auto") {
    ifstream probe(file.c_str());
    string line; nav = "vars";
    while(getline(probe, line))
      if(line.find("MSG_UDP") != string::npos &&
         (line.find("RMC,") != string::npos)) { nav = "nmea"; break; }
  }

  ifstream in(file.c_str());
  if(!in) { cerr << "cannot open " << file << endl; return 1; }

  string line, allstop = "?";
  bool autonomy = false;
  double next_tick = -1, last_detail = -1, lat = 0, lon = 0, t = 0;
  bool have_lat = false;
  int faults_in_auto = 0;
  NavGuardModel::State prev = NavGuardModel::OK;
  string prev_reason;
  double auto_start = -1;

  cout << "nav source: " << (nav == "nmea" ? "MSG_UDP NMEA" : "NAV_* variables") << endl;

  while(getline(in, line)) {
    if(line.empty() || line[0] == '%') continue;
    istringstream ss(line);
    string var, src;
    if(!(ss >> t >> var >> src)) continue;
    string val;
    getline(ss, val);
    size_t b = val.find_first_not_of(" \t");
    val = (b == string::npos) ? "" : val.substr(b);
    while(!val.empty() && (val[val.size()-1] == ' ' || val[val.size()-1] == '\r')) val.erase(val.size()-1);

    // 4 Hz evaluation (pNavGuard AppTick) up to this record's time
    if(next_tick < 0) next_tick = t;
    while(next_tick <= t) {
      m.setAutonomy(autonomy);
      NavGuardModel::State s = m.update(next_tick);
      if(s != prev || (verbose && m.reason() != prev_reason)) {
        const char* names[] = {"OK   ", "WARN ", "FAULT"};
        printf("%8.1f  %s  %-6s %s\n", next_tick, names[s],
               autonomy ? "AUTO" : "manual", m.reason().c_str());
        if(s == NavGuardModel::FAULT && prev != s && autonomy) {
          faults_in_auto++;
          printf("          -> pNavGuard stops autonomy here (%.1f s after start)\n",
                 next_tick - auto_start);
        }
      }
      if(verbose && next_tick - last_detail >= 1) {
        printf("            gps_spd=%.2f trk=%.0f hdg_err=%.0f\n",
               m.gpsSpeed(), m.gpsTrack(), m.headingError());
        last_detail = next_tick;
      }
      prev = s; prev_reason = m.reason();
      next_tick += 0.25;
    }

    if(var == "IVPHELM_ALLSTOP") {
      bool a = (val == "clear");
      if(a && !autonomy) {
        auto_start = t;
        printf("%8.1f  ----- autonomy starts (helm clear)\n", t);
      }
      if(!a && autonomy)
        printf("%8.1f  ----- autonomy ends (%s)\n", t, val.c_str());
      autonomy = a;
      allstop = val;
    }
    else if(var == "DESIRED_HEADING")
      m.desiredHeading(t, atof(val.c_str()));
    else if(nav == "nmea" && var == "MSG_UDP" && val.size() > 6 && val[0] == '$') {
      vector<string> f = split(val.substr(0, val.find('*')), ',');
      string type = f[0].size() >= 6 ? f[0].substr(3, 3) : "";
      if(type == "RMC" && f.size() >= 9) {
        m.gpsValid(t, f[2] != "V");
        double la, lo;
        if(nmeaDeg(f[3], f[4], la) && nmeaDeg(f[5], f[6], lo))
          m.position(t, la, lo);
        if(!f[7].empty()) m.sog(t, atof(f[7].c_str()) * kKnot);
        if(!f[8].empty()) m.cog(t, atof(f[8].c_str()));
      }
      else if(type == "HDT" && f.size() >= 2 && !f[1].empty())
        m.heading(t, atof(f[1].c_str()));
    }
    else if(nav == "vars") {
      if(var == "NAV_LAT") { lat = atof(val.c_str()); have_lat = true; }
      else if(var == "NAV_LONG" && have_lat) { lon = atof(val.c_str()); m.position(t, lat, lon); }
      else if(var == "NAV_SPEED") m.sog(t, atof(val.c_str()) * spd_scale);
      else if(var == "NAV_COG")   m.cog(t, atof(val.c_str()));
      else if(var == "NAV_HEADING") m.heading(t, atof(val.c_str()));
    }
  }
  printf("end of log at %.1f s; faults while in autonomy: %d\n", t, faults_in_auto);
  return 0;
}
