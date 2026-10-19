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
    else if(key == "CORRIDOR_UPDATE") handleCorridorUpdate(msg.GetString());
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
  bool active = m_corridor_on && m_waypoint_on && m_model.geometryValid() &&
                m_nav_x_known && m_nav_y_known;

  CorridorModel::Zone zone = m_model.update(m_nav_x, m_nav_y, active);
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

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

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
  Register("CORRIDOR_UPDATE", 0);
  Register("WAYPOINT", 0);
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
  m_msgs << actab.getFormattedString();

  return(true);
}
