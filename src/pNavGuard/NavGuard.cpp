/************************************************************/
/*    FILE: NavGuard.cpp                                    */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "ACTable.h"
#include "NavGuard.h"

using namespace std;

static const double kKnot = 0.514444;

NavGuard::NavGuard()
{
  m_spd_scale = 1.0;
  m_spd_units = "mps";
  m_stop_on_fault = true;
  m_nmea_var = "MSG_UDP";
  m_lat = m_lon = 0;
  m_lat_new = m_lon_new = false;
  m_posted_state = NavGuardModel::OK;
  m_last_post = m_last_stop = -1;
  m_faults = m_stops = 0;
}

bool NavGuard::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();
    double t = msg.GetTime();

    if(key == "NAV_LAT")          { m_lat = msg.GetDouble(); m_lat_new = true; }
    else if(key == "NAV_LONG")    { m_lon = msg.GetDouble(); m_lon_new = true; }
    else if(key == "NAV_SPEED")   m_model.sog(t, msg.GetDouble() * m_spd_scale);
    else if(key == "NAV_COG")     m_model.cog(t, msg.GetDouble());
    else if(key == "NAV_HEADING") m_model.heading(t, msg.GetDouble());
    else if(key == "DESIRED_HEADING") m_model.desiredHeading(t, msg.GetDouble());
    else if(key == "IVPHELM_ALLSTOP") m_allstop = msg.GetString();
    else if(key == m_nmea_var) {
      // $--RMC,time,status,...: status V = the receiver has no fix
      string s = msg.GetString();
      if(s.size() > 7 && s[0] == '$' && s.compare(3, 3, "RMC") == 0) {
        vector<string> f = parseString(s, ',');
        if(f.size() > 2)
          m_model.gpsValid(t, f[2] != "V");
      }
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);

    // iDivisorNMEA posts NAV_LAT then NAV_LONG for each sentence
    if(m_lat_new && m_lon_new) {
      m_model.position(t, m_lat, m_lon);
      m_lat_new = m_lon_new = false;
    }
  }
  return(true);
}

bool NavGuard::OnConnectToServer()
{
  registerVariables();
  return(true);
}

bool NavGuard::Iterate()
{
  AppCastingMOOSApp::Iterate();

  double now = MOOSTime();
  bool autonomy = (m_allstop == "clear");
  m_model.setAutonomy(autonomy);
  NavGuardModel::State s = m_model.update(now);
  string msg = m_model.reason();

  // Posted on a state change and otherwise once a second (iWebSocket
  // treats 5 s of silence as "monitor not reporting"); the reason's
  // numbers change every tick, so it is not posted on every change.
  if(s != m_posted_state || now - m_last_post >= 1.0) {
    if(s != m_posted_state) {
      if(s == NavGuardModel::FAULT) {
        m_faults++;
        m_last_fault = msg;
        reportRunWarning("Navigation FAULT: " + msg);
      }
      reportEvent("NAV_STATUS " + m_model.stateName() + (msg.empty() ? "" : ": " + msg));
    }
    Notify("NAV_STATUS", m_model.stateName());
    Notify("NAV_STATUS_MSG", msg);
    m_posted_state = s;
    m_last_post = now;
  }

  // In a fault while the helm drives: stop. Repeated every 2 s for as long
  // as both hold, so a mission started anyway is stopped again.
  if(m_stop_on_fault && s == NavGuardModel::FAULT && autonomy &&
     (m_last_stop < 0 || now - m_last_stop >= 2.0)) {
    Notify("NAV_GUARD_STOP", msg);
    m_last_stop = now;
    m_stops++;
  }
  if(!autonomy)
    m_last_stop = -1;

  AppCastingMOOSApp::PostReport();
  return(true);
}

// Speeds in the config are in nav_speed_units; the model wants m/s.
bool NavGuard::setSpeedParam(const string& param, double val)
{
  double mps = val * m_spd_scale;
  if(param == "min_speed")  return m_model.setMinSpeed(mps);
  if(param == "speed_tol")  return m_model.setSpeedTol(mps);
  if(param == "jump_speed") return m_model.setJumpSpeed(mps);
  return false;
}

bool NavGuard::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  // nav_speed_units first: the speed parameters depend on it
  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string line = *p;
    string param = tolower(biteStringX(line, '='));
    if(param == "nav_speed_units") {
      string v = tolower(line);
      if(v == "knots")    { m_spd_scale = kKnot; m_spd_units = "knots"; }
      else if(v == "mps") { m_spd_scale = 1.0;   m_spd_units = "mps"; }
      else reportConfigWarning("nav_speed_units must be knots or mps: " + line);
    }
  }

  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    double dval  = atof(value.c_str());
    bool num = isNumber(value);

    bool handled = false;
    if(param == "nav_speed_units")
      handled = true;
    else if(param == "stop_on_fault")
      handled = setBooleanOnString(m_stop_on_fault, value);
    else if(param == "nmea_var") {
      m_nmea_var = (tolower(value) == "none") ? "" : toupper(value);
      handled = true;
    }
    else if(!num)
      handled = false;
    else if(param == "min_speed" || param == "speed_tol" || param == "jump_speed")
      handled = setSpeedParam(param, dval);
    else if(param == "heading_tol")         handled = m_model.setHeadingTol(dval);
    else if(param == "heading_secs")        handled = m_model.setHeadingSecs(dval);
    else if(param == "speed_secs")          handled = m_model.setSpeedSecs(dval);
    else if(param == "freeze_secs")         handled = m_model.setFreezeSecs(dval);
    else if(param == "heading_freeze_secs") handled = m_model.setHeadingFreezeSecs(dval);
    else if(param == "stale_secs")          handled = m_model.setStaleSecs(dval);
    else if(param == "jump_min_dist")       handled = m_model.setJumpMinDist(dval);
    else if(param == "gps_flag_secs")       handled = m_model.setGpsFlagSecs(dval);
    else if(param == "follow_tol")          handled = m_model.setFollowTol(dval);
    else if(param == "follow_secs")         handled = m_model.setFollowSecs(dval);
    else if(param == "clear_secs")          handled = m_model.setClearSecs(dval);
    else if(param == "window_secs")         handled = m_model.setWindowSecs(dval);

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  registerVariables();
  return(true);
}

void NavGuard::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_LAT", 0);
  Register("NAV_LONG", 0);
  Register("NAV_SPEED", 0);
  Register("NAV_COG", 0);
  Register("NAV_HEADING", 0);
  Register("DESIRED_HEADING", 0);
  Register("IVPHELM_ALLSTOP", 0);
  if(!m_nmea_var.empty())
    Register(m_nmea_var, 0);
}

bool NavGuard::buildReport()
{
  double k = 1.0 / m_spd_scale;   // back to nav_speed_units
  m_msgs << "Speed units:   " << m_spd_units << endl;
  m_msgs << "Stop on fault: " << boolToString(m_stop_on_fault) << endl;
  m_msgs << "Helm:          " << (m_allstop.empty() ? "?" : m_allstop) << endl;
  m_msgs << endl;
  ACTable actab(2);
  actab << "Item | Value";
  actab.addHeaderLines();
  actab << "NAV_STATUS" << m_model.stateName();
  actab << "reason" << (m_model.reason().empty() ? "-" : m_model.reason());
  actab << "GPS speed (" + m_spd_units + ")" << (m_model.gpsSpeed() < 0 ? string("-") : doubleToString(m_model.gpsSpeed() * k, 1));
  actab << "GPS track (deg)" << (m_model.gpsTrack() < 0 ? string("-") : doubleToString(m_model.gpsTrack(), 0));
  actab << "heading vs track (deg)" << (m_model.headingError() < 0 ? string("-") : doubleToString(m_model.headingError(), 0));
  actab << "faults" << uintToString(m_faults);
  actab << "autonomy stops" << uintToString(m_stops);
  actab << "last fault" << (m_last_fault.empty() ? "-" : m_last_fault);
  m_msgs << actab.getFormattedString();
  return(true);
}
