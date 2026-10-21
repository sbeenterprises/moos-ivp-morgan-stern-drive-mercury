/************************************************************/
/*    FILE: TurnLimit.cpp                                   */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "TurnLimit.h"

using namespace std;

TurnLimit::TurnLimit()
{
  m_turn_speed = -1;
  m_waypoint = m_trailing = m_stationing = m_manual = false;
  m_x_known = m_y_known = m_hdg_known = false;
  m_nav_x = m_nav_y = m_nav_hdg = 0;
  m_posted = false;
  m_posted_once = false;
  m_limit_events = 0;
}

static bool isTrue(const CMOOSMsg& msg)
{
  if(msg.IsDouble())
    return msg.GetDouble() != 0;
  return tolower(stripBlankEnds(msg.GetString())) == "true";
}

bool TurnLimit::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if(key == "NAV_X")             { m_nav_x = msg.GetDouble(); m_x_known = true; }
    else if(key == "NAV_Y")        { m_nav_y = msg.GetDouble(); m_y_known = true; }
    else if(key == "NAV_HEADING")  { m_nav_hdg = msg.GetDouble(); m_hdg_known = true; }
    else if(key == "DESIRED_HEADING")
      m_model.setDesiredHeading(msg.GetDouble(), MOOSTime());
    else if(key == "WAYPOINT")     m_waypoint = isTrue(msg);
    else if(key == "TRAILING")     m_trailing = isTrue(msg);
    else if(key == "STATIONING")   m_stationing = isTrue(msg);
    else if(key == "MOOS_MANUAL_OVERRIDE") m_manual = isTrue(msg);
    else if(key == "WPT_UPDATE")   m_model.applyWptUpdate(msg.GetString());
    else if(key == "WPT_INDEX")    m_model.setWptIndex((int)msg.GetDouble());
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }
  return(true);
}

bool TurnLimit::OnConnectToServer()
{
  registerVariables();
  return(true);
}

void TurnLimit::postSpeed()
{
  if(m_turn_speed > 0)
    Notify("TURN_LIMIT_UPDATES", "max_speed=" + doubleToStringX(m_turn_speed, 2));
}

bool TurnLimit::Iterate()
{
  AppCastingMOOSApp::Iterate();

  bool autonomy = (m_waypoint || m_trailing || m_stationing) && !m_manual &&
                  (m_turn_speed > 0);
  bool limited = m_model.update(MOOSTime(), autonomy, m_x_known && m_y_known,
                                m_nav_x, m_nav_y, m_hdg_known, m_nav_hdg);

  if(!m_posted_once || limited != m_posted) {
    if(limited) {
      postSpeed();   // the cap's speed, in case the helm started after us
      m_limit_events++;
    }
    Notify("TURN_LIMIT", limited ? "true" : "false");
    m_posted = limited;
    m_posted_once = true;
  }
  if(m_model.reason() != m_posted_reason) {
    Notify("TURN_LIMIT_REASON", m_model.reason());
    m_posted_reason = m_model.reason();
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

bool TurnLimit::OnStartUp()
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
    bool num = isNumber(value);

    bool handled = false;
    if(param == "turn_speed" && num && dval > 0) {
      m_turn_speed = dval;
      handled = true;
    }
    else if(param == "heading_trigger" && num)
      handled = m_model.setHeadingTrigger(dval);
    else if(param == "heading_release" && num)
      handled = m_model.setHeadingRelease(dval);
    else if(param == "corner_dist" && num)
      handled = m_model.setCornerDist(dval);
    else if(param == "corner_angle" && num)
      handled = m_model.setCornerAngle(dval);
    else if(param == "hold_secs" && num)
      handled = m_model.setHoldSecs(dval);
    else if(param == "desired_stale_secs" && num)
      handled = m_model.setDesiredStaleSecs(dval);

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  if(m_turn_speed <= 0)
    reportConfigWarning("turn_speed not set: pTurnLimit will never limit the speed");

  registerVariables();
  postSpeed();
  return(true);
}

void TurnLimit::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("DESIRED_HEADING", 0);
  Register("WAYPOINT", 0);
  Register("TRAILING", 0);
  Register("STATIONING", 0);
  Register("MOOS_MANUAL_OVERRIDE", 0);
  Register("WPT_UPDATE", 0);
  Register("WPT_INDEX", 0);
}

bool TurnLimit::buildReport()
{
  m_msgs << "Turn speed:   " << (m_turn_speed > 0 ? doubleToStringX(m_turn_speed, 2) : "not set (never limits)") << endl;
  m_msgs << "Autonomy:     WAYPOINT=" << boolToString(m_waypoint)
         << " TRAILING=" << boolToString(m_trailing)
         << " STATIONING=" << boolToString(m_stationing)
         << " manual=" << boolToString(m_manual) << endl;
  m_msgs << "Route:        " << m_model.routeSize() << " points" << endl;
  m_msgs << endl;
  ACTable actab(2);
  actab << "Item | Value";
  actab.addHeaderLines();
  actab << "TURN_LIMIT" << boolToString(m_model.limited());
  actab << "reason" << (m_model.reason().empty() ? "-" : m_model.reason());
  actab << "heading error (deg)" << (m_model.headingError() < 0 ? string("-") : doubleToString(m_model.headingError(), 0));
  actab << "next waypoint (m)" << (m_model.distToWpt() < 0 ? string("-") : doubleToString(m_model.distToWpt(), 0));
  actab << "turn at it (deg)" << (m_model.cornerAngle() < 0 ? string("-") : doubleToString(m_model.cornerAngle(), 0));
  actab << "times limited" << uintToString(m_limit_events);
  m_msgs << actab.getFormattedString();
  return(true);
}
