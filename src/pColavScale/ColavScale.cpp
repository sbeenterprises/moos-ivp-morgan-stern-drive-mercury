/************************************************************/
/*    FILE: ColavScale.cpp                                  */
/************************************************************/

#include <cmath>
#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "ColavScale.h"

using namespace std;

ColavScale::ColavScale()
{
  m_nav_speed_factor     = 1.0;
  m_contact_speed_factor = 1.0;
  m_prefix            = "avdcol_";
  m_cpa_range         = 1500;
  m_contact_max_age   = 30;
  m_alert_repost_secs = 15;
  for(int i = 0; i < 4; i++)
    m_nav_known[i] = false;
  m_nav_x = m_nav_y = m_nav_hdg = m_nav_spd = 0;
  m_last_alert_post = 0;
  m_updates_posted  = 0;
}

bool ColavScale::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();
    if(key == "NAV_X")            { m_nav_x = msg.GetDouble(); m_nav_known[0] = true; }
    else if(key == "NAV_Y")       { m_nav_y = msg.GetDouble(); m_nav_known[1] = true; }
    else if(key == "NAV_HEADING") { m_nav_hdg = msg.GetDouble(); m_nav_known[2] = true; }
    else if(key == "NAV_SPEED")   { m_nav_spd = msg.GetDouble(); m_nav_known[3] = true; }
    else if(key == "NODE_REPORT")        handleNodeReport(msg.GetString());
    else if(key == "IVPHELM_LIFE_EVENT") handleLifeEvent(msg.GetString());
    else if(key == "COLAV_DIST")         handleColavDist(msg.GetString());
    else if(key == "BCM_ALERT_REQUEST") {
      // The helm (at start-up) or iWebSocket set the alert range to the
      // template's outer distance: put ours back on the next iteration.
      if(msg.GetSource() != GetAppName())
        m_last_alert_post = 0;
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }
  return(true);
}

// NAME=AIS123,X=..,Y=..,SPD=..,HDG=..,...
void ColavScale::handleNodeReport(const string& report)
{
  string name;
  Contact c = {0, 0, 0, 0, MOOSTime()};
  int have = 0;
  vector<string> fields = parseString(report, ',');
  for(size_t i = 0; i < fields.size(); i++) {
    string f = stripBlankEnds(fields[i]);
    string k = toupper(biteStringX(f, '='));
    if(k == "NAME")      name = f;
    else if(k == "X")    { c.x = atof(f.c_str()); have |= 1; }
    else if(k == "Y")    { c.y = atof(f.c_str()); have |= 2; }
    else if(k == "SPD")  { c.spd = atof(f.c_str()) * m_contact_speed_factor; have |= 4; }
    else if(k == "HDG")  { c.hdg = atof(f.c_str()); have |= 8; }
  }
  if(name != "" && have == 15)
    m_contacts[name] = c;
}

// iter=..,time=.., bname=avdcol_AIS123, btype=..., event=spawn|death, seed=...
void ColavScale::handleLifeEvent(const string& event)
{
  string bname, ev;
  vector<string> fields = parseString(event, ',');
  for(size_t i = 0; i < fields.size(); i++) {
    string f = stripBlankEnds(fields[i]);
    string k = biteStringX(f, '=');
    if(k == "bname")      bname = f;
    else if(k == "event") ev = f;
  }
  if(!strBegins(bname, m_prefix) || bname.size() <= m_prefix.size())
    return;
  string contact = bname.substr(m_prefix.size());
  if(ev == "spawn")
    m_live.insert(contact);
  else if(ev == "death" || ev == "abandon" || ev == "disable") {
    m_live.erase(contact);
    m_posted.erase(contact);
  }
}

// iWebSocket's echo of the UI's Calibration distances: they become the
// minimums. iWebSocket also just pushed them into every instance and set
// the alert range to its outer distance, so re-post ours.
void ColavScale::handleColavDist(const string& echo)
{
  vector<string> fields = parseString(echo, ',');
  for(size_t i = 0; i < fields.size(); i++) {
    string f = stripBlankEnds(fields[i]);
    string k = biteStringX(f, '=');
    double v = atof(f.c_str());
    if(k == "pwt_outer_dist" && v > 0)      m_model.setOuterMin(v);
    else if(k == "pwt_inner_dist" && v > 0) m_model.setInnerMin(v);
    else if(k == "completed_dist" && v > 0) m_cpa_range = v;
  }
  m_posted.clear();
  postAlertRange();
}

void ColavScale::postAlertRange()
{
  // The helm sets this from the template's pwt_outer_dist at start-up;
  // contacts must be alerted (their instance spawned) at our largest
  // distance or the scaling never gets the chance to act.
  double range = m_model.outerMax();
  if(range < m_model.outerMin())
    range = m_model.outerMin();
  double cpa = (m_cpa_range > range) ? m_cpa_range : range;
  Notify("BCM_ALERT_REQUEST", "id=" + m_prefix + ", alert_range=" +
         doubleToStringX(range, 1) + ", cpa_range=" + doubleToStringX(cpa, 1));
  m_last_alert_post = MOOSTime();
}

bool ColavScale::Iterate()
{
  AppCastingMOOSApp::Iterate();
  double now = MOOSTime();

  if(now - m_last_alert_post >= m_alert_repost_secs)
    postAlertRange();

  bool nav = m_nav_known[0] && m_nav_known[1] && m_nav_known[2] && m_nav_known[3];
  string summary;
  set<string>::iterator it;
  for(it = m_live.begin(); it != m_live.end(); ++it) {
    map<string, Contact>::iterator c = m_contacts.find(*it);
    if(!nav || c == m_contacts.end() || now - c->second.time > m_contact_max_age)
      continue;   // no data: the instance keeps what it has (template / last)
    ColavDist d = m_model.distances(m_nav_x, m_nav_y, m_nav_hdg,
                                    m_nav_spd * m_nav_speed_factor,
                                    c->second.x, c->second.y, c->second.hdg, c->second.spd);
    map<string, ColavDist>::iterator p = m_posted.find(*it);
    if(p == m_posted.end() || m_model.changed(p->second, d)) {
      // "name=" addresses this contact's existing instance only (it is in
      // m_live), so it updates it rather than spawning a new one.
      Notify("CONTACT_INFO", "name=" + *it +
             " # pwt_outer_dist=" + doubleToStringX(d.outer, 1) +
             " # pwt_inner_dist=" + doubleToStringX(d.inner, 1));
      m_posted[*it] = d;
      m_updates_posted++;
    }
    if(summary != "") summary += ",";
    summary += *it + "=" + doubleToString(m_posted[*it].outer, 0) + "/" +
               doubleToString(m_posted[*it].inner, 0);
  }
  if(summary != m_summary) {
    Notify("COLAV_SCALE", summary);
    m_summary = summary;
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

bool ColavScale::OnConnectToServer()
{
  registerVariables();
  return(true);
}

static bool speedFactor(const string& v, double& factor)
{
  string s = tolower(v);
  if(s == "knots" || s == "kn" || s == "kts") { factor = 0.514444; return true; }
  if(s == "mps" || s == "m/s")                 { factor = 1.0;      return true; }
  return false;
}

bool ColavScale::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  bool units_set = false;
  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    double dval  = atof(value.c_str());
    bool num = isNumber(value);

    bool handled = false;
    if(param == "outer_secs" && num)        handled = m_model.setOuterSecs(dval);
    else if(param == "inner_secs" && num)   handled = m_model.setInnerSecs(dval);
    else if(param == "outer_min" && num)    handled = m_model.setOuterMin(dval);
    else if(param == "inner_min" && num)    handled = m_model.setInnerMin(dval);
    else if(param == "outer_max" && num)    handled = m_model.setOuterMax(dval);
    else if(param == "inner_max" && num)    handled = m_model.setInnerMax(dval);
    else if(param == "change_frac" && num)  handled = m_model.setChangeFrac(dval);
    else if(param == "change_min" && num)   handled = m_model.setChangeMin(dval);
    else if(param == "nav_speed_units")     handled = units_set = speedFactor(value, m_nav_speed_factor);
    else if(param == "contact_speed_units") handled = speedFactor(value, m_contact_speed_factor);
    else if(param == "behavior_prefix" && value != "") { m_prefix = value; handled = true; }
    else if(param == "cpa_range" && num && dval > 0)   { m_cpa_range = dval; handled = true; }
    else if(param == "contact_max_age" && num && dval > 0) { m_contact_max_age = dval; handled = true; }
    else if(param == "alert_repost_secs" && num && dval > 0) { m_alert_repost_secs = dval; handled = true; }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  if(!units_set)
    reportConfigWarning("nav_speed_units not set (knots on the boat, mps in the sim): assuming mps");

  registerVariables();
  postAlertRange();
  return(true);
}

void ColavScale::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("NODE_REPORT", 0);
  Register("IVPHELM_LIFE_EVENT", 0);
  Register("COLAV_DIST", 0);
  Register("BCM_ALERT_REQUEST", 0);
}

bool ColavScale::buildReport()
{
  m_msgs << "outer = " << doubleToStringX(m_model.outerSecs(), 1) << " s x relative speed, "
         << doubleToStringX(m_model.outerMin(), 0) << ".." << doubleToStringX(m_model.outerMax(), 0) << " m" << endl;
  m_msgs << "inner = " << doubleToStringX(m_model.innerSecs(), 1) << " s x relative speed, "
         << doubleToStringX(m_model.innerMin(), 0) << ".." << doubleToStringX(m_model.innerMax(), 0) << " m" << endl;
  m_msgs << "Alert range (BCM_ALERT_REQUEST): " << doubleToStringX(m_model.outerMax(), 0) << " m" << endl;
  m_msgs << "Updates posted: " << m_updates_posted << endl << endl;
  ACTable actab(4);
  actab << "Contact | Rel speed (m/s) | Outer (m) | Inner (m)";
  actab.addHeaderLines();
  set<string>::iterator it;
  for(it = m_live.begin(); it != m_live.end(); ++it) {
    map<string, ColavDist>::iterator p = m_posted.find(*it);
    if(p == m_posted.end())
      actab << *it << "-" << "-" << "-";
    else
      actab << *it << doubleToString(p->second.rel_speed, 1)
            << doubleToString(p->second.outer, 0) << doubleToString(p->second.inner, 0);
  }
  m_msgs << actab.getFormattedString();
  return(true);
}
