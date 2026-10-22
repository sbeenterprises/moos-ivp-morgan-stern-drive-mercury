/************************************************************/
/*    FILE: Corridor.cpp                                    */
/*    DATE: 2026                                            */
/************************************************************/

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include "MBUtils.h"
#include "ACTable.h"
#include "Corridor.h"

using namespace std;

//---------------------------------------------------------
// Constructor

Corridor::Corridor()
{
  m_status_interval = 1.0;
  m_recover_move    = 1.0;

  m_nav_x = 0; m_nav_y = 0;
  m_nav_x_known = false; m_nav_y_known = false;
  m_corridor_on = false;
  m_waypoint_on = false;

  m_status_posted    = "";
  m_last_status_post = 0;
  m_was_active       = false;

  m_recover_posted = false;
  m_recover_x = 0; m_recover_y = 0;
  m_recover_posts = 0;

  m_tighten       = true;
  m_corr_capture  = 15;  m_corr_slip  = 30;  m_corr_lead  = 60;
  m_route_capture = 60;  m_route_slip = 90;  m_route_lead = 400;
  m_tight_applied = false;
  m_helm_manual   = false;

  m_turn_dist      = 60;
  m_turn_min_angle = 60;
  m_turn_on        = false;
  m_turn_posted    = false;
  m_turn_allowance = 0;

  m_slow_speed = -1;  m_turn_speed = -1;  m_recover_speed = -1;
  m_mission_speed = -1;
  m_posted_slow = -1; m_posted_turn = -1; m_posted_recover = -1;
  m_stop_recovers = false;
}

//---------------------------------------------------------
// Procedure: missionSpeedFrom()
//   The mission speed rides on WPT_UPDATE as "speed=<v>" (alone or among
//   other "param=value # ..." pairs).

bool Corridor::missionSpeedFrom(const string& wpt_update, double& spd)
{
  vector<string> parts = parseString(wpt_update, '#');
  for(unsigned int i=0; i<parts.size(); i++) {
    string val = stripBlankEnds(parts[i]);
    string key = tolower(stripBlankEnds(biteString(val, '=')));
    val = stripBlankEnds(val);
    if(key == "speed" && isNumber(val) && atof(val.c_str()) >= 0) {
      spd = atof(val.c_str());
      return(true);
    }
  }
  return(false);
}

//---------------------------------------------------------
// Procedure: postSpeeds()
//   Speeds for corridor_slow / corridor_turn / corridor_recover: the
//   configured value, never above the mission speed. Posted on change only.

void Corridor::postSpeeds()
{
  struct Item { double cfg; double* posted; const char* var; };
  Item items[] = {{m_slow_speed,    &m_posted_slow,    "CORRIDOR_SLOW_UPDATES"},
                  {m_turn_speed,    &m_posted_turn,    "CORRIDOR_TURN_UPDATES"},
                  {m_recover_speed, &m_posted_recover, "CORRIDOR_RECOVER_UPDATES"}};
  for(unsigned int i=0; i<3; i++) {
    if(items[i].cfg < 0)
      continue;                              // not configured: .bhv value
    double v = items[i].cfg;
    if(m_mission_speed >= 0 && m_mission_speed < v)
      v = m_mission_speed;                   // a cap, never a speed-up
    if(*items[i].posted >= 0 && fabs(v - *items[i].posted) < 0.01)
      continue;
    Notify(items[i].var, "speed=" + doubleToStringX(v, 2));
    *items[i].posted = v;
  }
}

//---------------------------------------------------------
// Procedure: followingSpec()
//   One WPT_UPDATE for waypt_survey ("param=value # param=value ...").

string Corridor::followingSpec(double capture, double slip, double lead) const
{
  ostringstream os;
  os << "capture_radius=" << doubleToStringX(capture, 1)
     << " # slip_radius=" << doubleToStringX(slip, 1)
     << " # lead=" << doubleToStringX(lead, 1);
  return(os.str());
}

//---------------------------------------------------------
// Procedure: parseFlag()
//   iWebSocket and the helm post these flags as strings ("true"), but a
//   double 1/0 from uPokeDB is accepted too.

bool Corridor::parseFlag(CMOOSMsg& msg)
{
  if(msg.IsDouble())
    return(msg.GetDouble() != 0);
  return(tolower(stripBlankEnds(msg.GetString())) == "true");
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool Corridor::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if(key == "NAV_X")          { m_nav_x = msg.GetDouble(); m_nav_x_known = true; }
    else if(key == "NAV_Y")     { m_nav_y = msg.GetDouble(); m_nav_y_known = true; }
    else if(key == "CORRIDOR")  m_corridor_on = parseFlag(msg);
    else if(key == "WAYPOINT")  m_waypoint_on = parseFlag(msg);
    else if(key == "IVPHELM_ALLSTOP") {
      bool manual = (msg.GetString() == "ManualOverride");
      if(m_helm_manual && !manual) {
        if(!m_following_posted.empty()) {
          Notify("WPT_UPDATE", m_following_posted);
          reportEvent("Route following re-sent: helm took control");
        }
        // the corridor speeds go again on the next postSpeeds()
        m_posted_slow = m_posted_turn = m_posted_recover = -1;
      }
      m_helm_manual = manual;
    }
    else if(key == "CORRIDOR_UPDATE") handleCorridorUpdate(msg.GetString());
    else if(key == "WPT_UPDATE") {
      double spd = 0;
      if(missionSpeedFrom(msg.GetString(), spd))
        m_mission_speed = spd;
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: handleCorridorUpdate()
//   width=<w>,pts={x1,y1:x2,y2:...}
//   A bad update clears the corridor rather than keeping the old one: the
//   operator asked for something else, so the old shape is no longer what
//   they expect the boat to respect. iWebSocket validates first, so this
//   only happens on a hand-poked value.

void Corridor::handleCorridorUpdate(const string& sval)
{
  m_last_spec = sval;
  string err;
  if(m_model.setGeometryFromSpec(sval, err)) {
    m_last_error = "";
    reportEvent("Corridor set: width=" + doubleToStringX(m_model.width(), 1) +
                ", " + uintToString(m_model.size()) + " points");
  }
  else {
    m_last_error = err;
    reportRunWarning("CORRIDOR_UPDATE rejected (" + err + "): " + sval);
  }
  // A new geometry restarts the recover point bookkeeping.
  m_recover_posted = false;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool Corridor::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool Corridor::Iterate()
{
  AppCastingMOOSApp::Iterate();

  double now = MOOSTime();

  // A corridor needs the route followed closely: with the route's normal
  // 60 m capture radius and 400 m lead the boat skips short legs and cuts
  // corners by more than a corridor's half width. Tighten while it is on,
  // restore the normal values when it goes off. Nothing is posted while no
  // corridor has been sent, so missions without one are unchanged.
  if(m_tighten && m_corridor_on && !m_tight_applied) {
    m_following_posted = followingSpec(m_corr_capture, m_corr_slip, m_corr_lead);
    Notify("WPT_UPDATE", m_following_posted);
    m_tight_applied = true;
    reportEvent("Route following tightened for the corridor");
  }
  else if(m_tight_applied && !m_corridor_on) {
    m_following_posted = followingSpec(m_route_capture, m_route_slip, m_route_lead);
    Notify("WPT_UPDATE", m_following_posted);
    m_tight_applied = false;
    reportEvent("Route following restored");
  }

  postSpeeds();

  bool active = m_corridor_on && m_waypoint_on && m_model.geometryValid() &&
                m_nav_x_known && m_nav_y_known;

  CorridorModel::Zone zone = m_model.update(m_nav_x, m_nav_y, active);
  // stop_action = recover: far outside is driven like "recovering" (steer
  // back at the recover speed) instead of stopping without steerage.
  if(zone == CorridorModel::ZONE_STOP && m_stop_recovers)
    zone = CorridorModel::ZONE_RECOVERING;
  string status = CorridorModel::zoneName(zone);

  if(zone != CorridorModel::ZONE_NONE) {
    Notify("XTE_NOW", m_model.xte());

    // The recover point is posted before CORRIDOR_STATUS so that, in the
    // iteration that switches to "recovering", corridor_recover already has
    // its target when its condition turns true.
    if(zone == CorridorModel::ZONE_RECOVERING) {
      double rx = 0, ry = 0;
      if(m_model.recoverPoint(rx, ry)) {
        bool moved = !m_recover_posted ||
          (hypot(rx - m_recover_x, ry - m_recover_y) >= m_recover_move);
        if(moved || status != m_status_posted) {
          ostringstream os;
          os << fixed << setprecision(1) << "points=" << rx << "," << ry;
          Notify("CORRIDOR_RECOVER_UPDATES", os.str());
          m_recover_x = rx;
          m_recover_y = ry;
          m_recover_posted = true;
          m_recover_posts++;
        }
      }
    }
  }
  else if(m_was_active) {
    // Leaving the corridor (mission over, corridor off): zero the readout
    // once so the UI does not keep showing a stale distance.
    Notify("XTE_NOW", 0.0);
  }

  // Sharp turn ahead (or being made): corridor_turn slows the boat so the
  // turn stays inside the corridor. Only while the corridor is active.
  bool turn = (zone != CorridorModel::ZONE_NONE) &&
    m_model.nearSharpTurn(m_nav_x, m_nav_y, m_turn_dist, m_turn_min_angle);
  if(!m_turn_posted || turn != m_turn_on) {
    if(m_turn_posted || turn) {
      Notify("CORRIDOR_TURN", turn ? "true" : "false");
      if(turn != m_turn_on)
        reportEvent(string("CORRIDOR_TURN ") + (turn ? "true" : "false"));
    }
    m_turn_on = turn;
    m_turn_posted = true;
  }
  m_was_active = (zone != CorridorModel::ZONE_NONE);

  if(status != m_status_posted || (now - m_last_status_post) >= m_status_interval) {
    if(status != m_status_posted)
      reportEvent("CORRIDOR_STATUS " + (m_status_posted.empty() ? string("-") : m_status_posted) +
                  " -> " + status + " (xte=" + doubleToStringX(m_model.xte(), 1) + ")");
    Notify("CORRIDOR_STATUS", status);
    m_status_posted = status;
    m_last_status_post = now;
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool Corridor::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    double dval  = atof(value.c_str());
    bool handled = false;

    if(param == "hysteresis")
      handled = isNumber(value) && m_model.setHysteresis(dval);
    else if(param == "slow_frac")
      handled = isNumber(value) && m_model.setSlowFrac(dval);
    else if(param == "stop_factor")
      handled = isNumber(value) && m_model.setStopFactor(dval);
    else if(param == "recover_lead")
      handled = isNumber(value) && m_model.setRecoverLead(dval);
    else if(param == "status_interval" && isNumber(value) && dval > 0) {
      m_status_interval = dval;
      handled = true;
    }
    else if(param == "recover_move" && isNumber(value) && dval >= 0) {
      m_recover_move = dval;
      handled = true;
    }
    else if(param == "turn_slow_dist" && isNumber(value) && dval >= 0) {
      m_turn_dist = dval;
      handled = true;
    }
    else if(param == "turn_min_angle" && isNumber(value) && dval > 0 && dval <= 180) {
      m_turn_min_angle = dval;
      handled = true;
    }
    else if(param == "turn_allowance" && isNumber(value) && dval >= 0) {
      m_turn_allowance = dval;
      handled = true;
    }
    else if((param == "slow_speed" || param == "turn_speed" || param == "recover_speed") &&
            isNumber(value) && dval >= 0) {
      double *dst = (param == "slow_speed") ? &m_slow_speed :
                    (param == "turn_speed") ? &m_turn_speed : &m_recover_speed;
      *dst = dval;
      handled = true;
    }
    else if(param == "stop_action") {
      string v = tolower(value);
      if(v == "stop" || v == "recover") {
        m_stop_recovers = (v == "recover");
        handled = true;
      }
    }
    else if(param == "tighten_following")
      handled = setBooleanOnString(m_tighten, value);
    else if(isNumber(value) && dval > 0 &&
            (param == "corridor_capture_radius" || param == "corridor_slip_radius" ||
             param == "corridor_lead" || param == "route_capture_radius" ||
             param == "route_slip_radius" || param == "route_lead")) {
      double *dst = (param == "corridor_capture_radius") ? &m_corr_capture :
                    (param == "corridor_slip_radius")    ? &m_corr_slip :
                    (param == "corridor_lead")           ? &m_corr_lead :
                    (param == "route_capture_radius")    ? &m_route_capture :
                    (param == "route_slip_radius")       ? &m_route_slip : &m_route_lead;
      *dst = dval;
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  if(!m_model.setTurnAllowance(m_turn_dist, m_turn_min_angle, m_turn_allowance))
    reportConfigWarning("turn_allowance / turn_slow_dist / turn_min_angle out of range");

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void Corridor::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("CORRIDOR", 0);
  Register("IVPHELM_ALLSTOP", 0);   // re-send following when the helm takes control
  Register("CORRIDOR_UPDATE", 0);
  Register("WAYPOINT", 0);
  Register("WPT_UPDATE", 0);   // mission speed, to cap the corridor speeds
}

//---------------------------------------------------------
// Procedure: buildReport()

bool Corridor::buildReport()
{
  m_msgs << "Config: slow_frac=" << m_model.slowFrac()
         << ", stop_factor=" << m_model.stopFactor()
         << ", hysteresis=" << m_model.hysteresis()
         << ", recover_lead=" << m_model.recoverLead() << endl;
  m_msgs << endl;

  ACTable actab(2);
  actab << "Item | Value";
  actab.addHeaderLines();
  actab << "CORRIDOR" << boolToString(m_corridor_on);
  actab << "WAYPOINT" << boolToString(m_waypoint_on);
  actab << "Geometry valid" << boolToString(m_model.geometryValid());
  if(m_model.geometryValid()) {
    actab << "Width / points" << doubleToStringX(m_model.width(), 1) + " / " +
                                 uintToString(m_model.size());
    actab << "Limits in/slow/recover (m)" <<
      doubleToStringX(m_model.slowLimit(), 1) + " / " +
      doubleToStringX(m_model.halfWidth(), 1) + " / " +
      doubleToStringX(m_model.stopLimit(), 1);
  }
  actab << "Last error" << m_last_error;
  actab << "NAV_X, NAV_Y" << doubleToStringX(m_nav_x, 1) + ", " + doubleToStringX(m_nav_y, 1);
  actab << "CORRIDOR_STATUS" << m_status_posted;
  actab << "XTE_NOW" << doubleToStringX(m_model.xte(), 1);
  actab << "Recover point" << (m_recover_posted ?
     doubleToStringX(m_recover_x, 1) + ", " + doubleToStringX(m_recover_y, 1) : string("-"));
  actab << "Recover posts" << uintToString(m_recover_posts);
  actab << "Route following" << (m_tight_applied ? string("tight (corridor)") : string("normal"));
  actab << "Sharp turn ahead" << boolToString(m_turn_on);
  actab << "Turn allowance (m)" << doubleToStringX(m_model.turnAllowance(), 1) +
           " (now " + doubleToStringX(m_model.allowanceApplied(), 1) + ")";
  actab << "Speeds slow/turn/recover" <<
    (m_posted_slow < 0 ? string("bhv") : doubleToStringX(m_posted_slow, 2)) + " / " +
    (m_posted_turn < 0 ? string("bhv") : doubleToStringX(m_posted_turn, 2)) + " / " +
    (m_posted_recover < 0 ? string("bhv") : doubleToStringX(m_posted_recover, 2)) +
    "  (mission " + (m_mission_speed < 0 ? string("?") : doubleToStringX(m_mission_speed, 2)) + ")";
  actab << "Far outside" << string(m_stop_recovers ? "steer back (recover)" : "stop");
  m_msgs << actab.getFormattedString();

  return(true);
}
