/************************************************************/
/*    NAME: DL                                              */
/*    FILE: ColavEval.cpp                                   */
/*    DATE: 2026                                            */
/************************************************************/

#include <iterator>
#include <cmath>
#include <cstdlib>
#include "MBUtils.h"
#include "ACTable.h"
#include "AngleUtils.h"
#include "CPAEngine.h"
#include "ColavEval.h"

using namespace std;

static const double NA = -1;

//---------------------------------------------------------
// Encounter constructor

ColavEval::Encounter::Encounter()
{
  seq = 0;  mmsi = 0;  is_test = false;
  state = "live";

  t_placed = NA; t_detect = NA; t_active = NA; t_cpa = NA; t_end = NA;
  rng_detect = NA; rng_active = NA;

  cpa_pred = NA; tcpa_pred = NA;
  min_range = -1;             // -1 until the first sample
  t_min = NA; x_min = 0; y_min = 0;

  range = NA; cpa_live = NA; tcpa_live = NA;

  apwt_max = 0;

  hdg_at_detect = 0;
  hdg_at_active = 0;
  deflection_max = 0;
  des_deflection_max = 0;
  deflection_valid = true;
  wpt_at_active = 0;

  spd_at_detect = 0;
  spd_min = -1;

  dur_active = 0;
  active_since = NA;
  active_spans = 0;

  alpha = NA; beta = NA;
  ucd_cpa = NA;
  verdict = "";
}

//---------------------------------------------------------
// Constructor

ColavEval::ColavEval()
{
  m_collision_range = 20;
  m_near_miss_range = 60;
  m_report_interval = 0.5;
  m_cpa_confirm     = 15;
  m_max_records     = 6;
  m_test_mmsi_low   = 366999900;
  m_test_mmsi_high  = 366999999;

  m_osx = 0; m_osy = 0; m_osh = 0; m_osv = 0; m_des_hdg = 0;
  m_os_known = false;
  m_osv_known = false;
  m_wpt_index = 0;
  m_waypoint_active = false;
  m_test_mode = false;

  m_last_post = 0;
  m_seq_next  = 0;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ColavEval::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if(key == "NAV_X")            { m_osx = msg.GetDouble(); m_os_known = true; }
    else if(key == "NAV_Y")       { m_osy = msg.GetDouble(); m_os_known = true; }
    else if(key == "NAV_HEADING") { m_osh = msg.GetDouble(); }
    else if(key == "NAV_SPEED")   { m_osv = msg.GetDouble(); m_osv_known = true; }
    else if(key == "DESIRED_HEADING") { m_des_hdg = msg.GetDouble(); }
    else if(key == "WPT_INDEX")   { m_wpt_index = msg.GetDouble(); }
    else if(key == "WAYPOINT")    { m_waypoint_active = (tolower(msg.GetString()) == "true"); }
    else if(key == "COLAV_TEST") {
      m_test_mode = (tolower(msg.GetString()) == "true");
      // Echoing this back is what tells the UI an evaluator is actually present
      // - without it the toggle would happily switch on against nothing.
      Notify("COLAV_TEST_ACK", m_test_mode ? "true" : "false");
    }
    else if(key == "NODE_REPORT")        handleNodeReport(msg.GetString());
    else if(key == "TRAFFIC_SPAWNED")    handleTrafficSpawned(msg.GetString());
    else if(key == "COLAV_BHV_SPAWN")    handleBhvSpawn(msg.GetString());
    else if(key == "COLAV_BHV_ACTIVE")   handleBhvActive(msg.GetString());
    else if(key == "COLAV_BHV_INACTIVE") handleBhvInactive(msg.GetString());
    else if(key == "COLAV_BHV_CPA")      handleBhvCpa(msg.GetString());
    else if(key == "COLAV_BHV_END")      handleBhvEnd(msg.GetString());
    else if(key == "COLAV_BHV_TRACE")    handleBhvTrace(msg.GetString());
    else if(key == "IVPHELM_LIFE_EVENT") handleLifeEvent(msg.GetString());
    else if(key == "UCD_REPORT")         handleUcdReport(msg.GetString());
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ColavEval::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool ColavEval::Iterate()
{
  AppCastingMOOSApp::Iterate();

  double now = MOOSTime();
  for(unsigned int i=0; i<m_encounters.size(); i++) {
    if(m_encounters[i].state != "done")
      updateLive(m_encounters[i], now);
  }

  if((now - m_last_post) >= m_report_interval) {
    m_last_post = now;
    postMetrics();
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool ColavEval::OnStartUp()
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
    bool handled = false;

    if(param == "collision_range") {
      m_collision_range = atof(value.c_str());
      handled = true;
    }
    else if(param == "near_miss_range") {
      m_near_miss_range = atof(value.c_str());
      handled = true;
    }
    else if(param == "cpa_confirm") {
      m_cpa_confirm = atof(value.c_str());
      handled = true;
    }
    else if(param == "report_interval") {
      m_report_interval = atof(value.c_str());
      handled = true;
    }
    else if(param == "max_records") {
      m_max_records = (unsigned int) atoi(value.c_str());
      handled = true;
    }
    else if(param == "test_mmsi_low") {
      m_test_mmsi_low = (unsigned int) atol(value.c_str());
      handled = true;
    }
    else if(param == "test_mmsi_high") {
      m_test_mmsi_high = (unsigned int) atol(value.c_str());
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

void ColavEval::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("DESIRED_HEADING", 0);
  Register("WPT_INDEX", 0);
  Register("WAYPOINT", 0);
  Register("NODE_REPORT", 0);
  Register("TRAFFIC_SPAWNED", 0);
  Register("COLAV_TEST", 0);
  Register("COLAV_BHV_SPAWN", 0);
  Register("COLAV_BHV_ACTIVE", 0);
  Register("COLAV_BHV_INACTIVE", 0);
  Register("COLAV_BHV_CPA", 0);
  Register("COLAV_BHV_END", 0);
  Register("COLAV_BHV_TRACE", 0);
  Register("IVPHELM_LIFE_EVENT", 0);
  Register("UCD_REPORT", 0);
}

//---------------------------------------------------------
// Helpers

static string fieldOf(const string& spec, const string& key)
{
  return(tokStringParse(spec, key, ',', '='));
}

static double numOf(const string& spec, const string& key, double dflt)
{
  string s = fieldOf(spec, key);
  if(s == "")
    return(dflt);
  return(atof(s.c_str()));
}

/** JSON has no comment on what a bare quote does to a parser, so take none. */
static string jsonSafe(const string& in)
{
  string out;
  for(unsigned int i=0; i<in.size(); i++) {
    char ch = in[i];
    if((ch == '"') || (ch == '\\') || (ch < 32))
      continue;
    out += ch;
  }
  return(out);
}

//---------------------------------------------------------
// Procedure: findOpen() / openEncounter()

ColavEval::Encounter* ColavEval::findOpen(const string& cn)
{
  for(int i=(int)m_encounters.size()-1; i>=0; i--) {
    if((m_encounters[i].cn == cn) && (m_encounters[i].state != "done"))
      return(&m_encounters[i]);
  }
  return(0);
}

ColavEval::Encounter* ColavEval::openEncounter(const string& cn)
{
  Encounter* open = findOpen(cn);
  if(open)
    return(open);

  Encounter e;
  e.cn  = cn;
  e.seq = ++m_seq_next;

  // Contacts are named AIS<mmsi> by pTrafficSim and iDivisorNMEA alike, so the
  // MMSI - and with it whether this is a target the operator placed - comes out
  // of the name with no shared state to keep in sync.
  if(cn.substr(0,3) == "AIS") {
    e.mmsi = (unsigned int) atol(cn.substr(3).c_str());
    e.is_test = (e.mmsi >= m_test_mmsi_low) && (e.mmsi <= m_test_mmsi_high);
  }

  map<string,double>::iterator q = m_placed.find(cn);
  if(q != m_placed.end())
    e.t_placed = q->second;

  m_encounters.push_back(e);

  // Trim to max_records, but not blindly oldest-first: the record the operator
  // actually cares about is the target they just placed, and with four scenario
  // contacts alerting as well it would be the first thing pushed out. Drop a
  // finished scenario encounter first, then any finished one, and only fall
  // back to the plain oldest if everything on the list is still open.
  while(m_encounters.size() > m_max_records) {
    unsigned int victim = 0;
    bool found = false;
    for(unsigned int i=0; i<m_encounters.size() && !found; i++) {
      if((m_encounters[i].state == "done") && !m_encounters[i].is_test) {
        victim = i;
        found = true;
      }
    }
    for(unsigned int i=0; i<m_encounters.size() && !found; i++) {
      if(m_encounters[i].state == "done") {
        victim = i;
        found = true;
      }
    }
    m_encounters.erase(m_encounters.begin() + victim);
  }

  return(&m_encounters.back());
}

//---------------------------------------------------------
// Procedure: noteMode()

void ColavEval::noteMode(Encounter& e, const string& mode)
{
  if((mode == "") || (mode == e.mode))
    return;
  e.mode = mode;
  if(e.mode_history != "")
    e.mode_history += " ";
  e.mode_history += mode;
}

//---------------------------------------------------------
// Behaviour event handlers
//
// Every one of these arrives as a comma-separated key=value spec built by a
// flag on the avdcol_ template in alpha.bhv.

void ColavEval::handleBhvSpawn(const string& sval)
{
  string cn = fieldOf(sval, "cn");
  if(cn == "")
    return;

  Encounter* e = openEncounter(cn);
  e->t_detect      = numOf(sval, "utc", MOOSTime());
  e->rng_detect    = numOf(sval, "rng", NA);
  e->hdg_at_detect = numOf(sval, "osh", m_osh);
  e->spd_at_detect = numOf(sval, "osv", m_osv);

  // Predicted CPA the moment the behaviour picked the contact up, from the
  // geometry in the flag itself. Computed here rather than asked for as a
  // macro: $[CPA] is accepted by postFlag() but never expanded, so it would
  // post a silent 0.0.
  double cnx = numOf(sval, "cnx", 0), cny = numOf(sval, "cny", 0);
  double cnh = numOf(sval, "cnh", 0), cnv = numOf(sval, "cnv", 0);
  double osx = numOf(sval, "osx", m_osx), osy = numOf(sval, "osy", m_osy);
  double osh = e->hdg_at_detect, osv = e->spd_at_detect;
  if(osv > 0.01) {
    CPAEngine engine(cny, cnx, cnh, cnv, osy, osx);
    e->cpa_pred  = engine.evalCPA(osh, osv, 600);
    e->tcpa_pred = engine.evalTimeCPA(osh, osv, 600);
  }
}

void ColavEval::handleBhvActive(const string& sval)
{
  string cn = fieldOf(sval, "cn");
  Encounter* e = findOpen(cn);
  if(!e)
    return;

  double utc = numOf(sval, "utc", MOOSTime());
  // activeflag fires on every entry into ACTIVE, so latch the first one as the
  // reaction instant and count the rest as separate spans.
  if(e->t_active < 0) {
    e->t_active   = utc;
    e->rng_active = numOf(sval, "rng", NA);
    // Deflection is measured from here on, against this heading.
    e->hdg_at_active = numOf(sval, "osh", m_osh);
    e->wpt_at_active = m_wpt_index;
  }
  if(e->active_since < 0) {
    e->active_since = utc;
    e->active_spans++;
  }
  noteMode(*e, fieldOf(sval, "mode"));
  double apwt = numOf(sval, "apwt", 0);
  if(apwt > e->apwt_max)
    e->apwt_max = apwt;
}

void ColavEval::handleBhvTrace(const string& sval)
{
  string cn = fieldOf(sval, "cn");
  Encounter* e = findOpen(cn);
  if(!e)
    return;
  noteMode(*e, fieldOf(sval, "mode"));
  double apwt = numOf(sval, "apwt", 0);
  if(apwt > e->apwt_max)
    e->apwt_max = apwt;
}

//---------------------------------------------------------
// Procedure: handleBhvInactive()
//   inactiveflag fires from two call sites in BehaviorSet - the active->inactive
//   transition and entry into idle - so closing the span only when one is
//   actually open is what keeps the accounting straight.

void ColavEval::handleBhvInactive(const string& sval)
{
  string cn = fieldOf(sval, "cn");
  Encounter* e = findOpen(cn);
  if(!e)
    return;
  if(e->active_since > 0) {
    e->dur_active += (numOf(sval, "utc", MOOSTime()) - e->active_since);
    e->active_since = NA;
  }
  noteMode(*e, fieldOf(sval, "mode"));
}

void ColavEval::handleBhvCpa(const string& sval)
{
  string cn = fieldOf(sval, "cn");
  Encounter* e = findOpen(cn);
  if(!e)
    return;

  // cpa_reached() is not latched and its threshold is 5 cm, so range jitter can
  // trip it more than once. Take the first, and report the achieved minimum
  // from our own tracking regardless.
  if(e->t_cpa < 0) {
    e->t_cpa = numOf(sval, "utc", MOOSTime());
    e->alpha = numOf(sval, "oscn", NA);
    e->beta  = numOf(sval, "cnos", NA);
  }
  // Deliberately does NOT flip the state. cpa_reached() is unlatched with a 5 cm
  // threshold and runs on an extrapolated contact, so a boat still settling onto
  // its leg trips it while the encounter is very much still closing. The state
  // moves on confirmed ground truth in updateLive() instead; this flag only
  // records the behaviour's own view of when it saw its CPA.
  noteMode(*e, fieldOf(sval, "mode"));
}

void ColavEval::handleBhvEnd(const string& sval)
{
  string cn = fieldOf(sval, "cn");
  Encounter* e = findOpen(cn);
  if(!e)
    return;
  noteMode(*e, fieldOf(sval, "mode"));
  closeEncounter(*e, numOf(sval, "utc", MOOSTime()), "behaviour_complete");
}

//---------------------------------------------------------
// Procedure: handleLifeEvent()
//   The authoritative death notice, and the only place the post_mortem shows up.
//
//   Format note: LifeEvent::getSpec() emits 'iter=' and the helm appends a
//   second one, and post_mortem/seed contain '#' and spaces - so parse by comma
//   and never split on '#'.

void ColavEval::handleLifeEvent(const string& sval)
{
  if(fieldOf(sval, "event") != "death")
    return;
  string bname = fieldOf(sval, "bname");
  if(bname.substr(0,7) != "avdcol_")
    return;

  string cn = bname.substr(7);
  Encounter* e = findOpen(cn);
  if(!e)
    return;

  e->post_mortem = fieldOf(sval, "post_mortem");
  closeEncounter(*e, MOOSTime(), "behaviour_death");
}

//---------------------------------------------------------
// Procedure: handleNodeReport()
//   Ground truth for the achieved CPA. The behaviour's own range runs on an
//   extrapolated contact position, so it is not the number to report.

void ColavEval::handleNodeReport(const string& sval)
{
  string name = tokStringParse(sval, "NAME", ',', '=');
  if(name == "")
    return;

  CnState st;
  st.x     = numOf(sval, "X", 0);
  st.y     = numOf(sval, "Y", 0);
  st.hdg   = numOf(sval, "HDG", 0);
  st.spd   = numOf(sval, "SPD", 0);
  st.utc   = MOOSTime();
  st.valid = true;
  m_contacts[name] = st;
}

void ColavEval::handleTrafficSpawned(const string& sval)
{
  if(fieldOf(sval, "ok") != "true")
    return;
  string name = fieldOf(sval, "name");
  if(name != "")
    m_placed[name] = numOf(sval, "utc", MOOSTime());
}

//---------------------------------------------------------
// Procedure: handleUcdReport()
//   uFldCollisionDetect, when it is running, is an independent measurement of
//   the same achieved CPA. Folding it in next to our own number means a
//   disagreement is visible rather than invisible.

void ColavEval::handleUcdReport(const string& sval)
{
  string v1 = fieldOf(sval, "vname1");
  string v2 = fieldOf(sval, "vname2");
  double cpa = numOf(sval, "cpa", NA);
  if(cpa < 0)
    return;

  for(int i=(int)m_encounters.size()-1; i>=0; i--) {
    if((m_encounters[i].cn == v1) || (m_encounters[i].cn == v2)) {
      m_encounters[i].ucd_cpa = cpa;
      return;
    }
  }
}

//---------------------------------------------------------
// Procedure: updateLive()

void ColavEval::updateLive(Encounter& e, double now)
{
  map<string,CnState>::iterator q = m_contacts.find(e.cn);
  if(!m_os_known || (q == m_contacts.end()) || !q->second.valid)
    return;
  const CnState& cn = q->second;

  double rng = hypot(cn.x - m_osx, cn.y - m_osy);
  e.range = rng;

  if((e.min_range < 0) || (rng < e.min_range)) {
    e.min_range = rng;
    e.t_min = now;
    e.x_min = (cn.x + m_osx) / 2;
    e.y_min = (cn.y + m_osy) / 2;
  }
  // The encounter is past its closest point once the range has opened by a
  // margin that range noise cannot account for. Measured here, on real
  // positions, so the panel never announces a provisional result while the
  // contact is in fact still closing.
  else if((e.state == "live") && (rng > (e.min_range + m_cpa_confirm)))
    e.state = "past_cpa";

  if(m_osv > 0.01) {
    CPAEngine engine(cn.y, cn.x, cn.hdg, cn.spd, m_osy, m_osx);
    e.cpa_live  = engine.evalCPA(m_osh, m_osv, 600);
    e.tcpa_live = engine.evalTimeCPA(m_osh, m_osv, 600);
  }

  if(m_osv_known && ((e.spd_min < 0) || (m_osv < e.spd_min)))
    e.spd_min = m_osv;

  // Only count deflection once the behaviour actually has the helm, and only
  // against the heading it started from. Before that the boat is steering its
  // route, and a leg turn counted as avoidance would flatter the result. Even
  // then, a waypoint change mid-encounter muddies it - latch that doubt rather
  // than reporting a number that quietly means something else.
  if(e.t_active > 0) {
    if(m_wpt_index != e.wpt_at_active)
      e.deflection_valid = false;
    double d = fabs(angle180(m_osh - e.hdg_at_active));
    if(d > e.deflection_max)
      e.deflection_max = d;
    double dd = fabs(angle180(m_des_hdg - e.hdg_at_active));
    if(dd > e.des_deflection_max)
      e.des_deflection_max = dd;
  }
}

//---------------------------------------------------------
// Procedure: closeEncounter()

void ColavEval::closeEncounter(Encounter& e, double now, const string& reason)
{
  if(e.state == "done")
    return;

  if(e.active_since > 0) {
    e.dur_active += (now - e.active_since);
    e.active_since = NA;
  }

  e.t_end  = now;
  e.state  = "done";
  if(e.end_reason == "")
    e.end_reason = reason;

  if(e.min_range < 0)
    e.verdict = "no_data";
  else if(e.min_range <= m_collision_range)
    e.verdict = "collision";
  else if(e.min_range <= m_near_miss_range)
    e.verdict = "near_miss";
  else
    e.verdict = "clear";

  string spec = encounterSpec(e);
  Notify("COLAV_ENCOUNTER", spec);

  // alogeplot and alogcd both read this, so a finished run can be replayed with
  // the stock tools instead of only through the UI.
  if(e.min_range >= 0) {
    string enc = "v1=" + string("os") + ",v2=" + e.cn;
    enc += ",cpa=" + doubleToString(e.min_range, 2);
    enc += ",x="   + doubleToString(e.x_min, 1);
    enc += ",y="   + doubleToString(e.y_min, 1);
    enc += ",id="  + uintToString(e.seq);
    if(e.alpha >= 0) enc += ",alpha=" + doubleToString(e.alpha, 1);
    if(e.beta  >= 0) enc += ",beta="  + doubleToString(e.beta, 1);
    Notify("ENCOUNTER_SUMMARY", enc);
  }

  postMetrics();
}

//---------------------------------------------------------
// Procedure: encounterSpec()
//   Flat key=value, for the log and for uXMS.

string ColavEval::encounterSpec(const Encounter& e) const
{
  string s = "cn=" + e.cn;
  s += ",seq="       + uintToString(e.seq);
  s += ",test="      + string(e.is_test ? "true" : "false");
  s += ",state="     + e.state;
  s += ",verdict="   + e.verdict;
  s += ",min_range=" + doubleToString(e.min_range, 2);
  s += ",cpa_pred="  + doubleToString(e.cpa_pred, 2);
  s += ",rng_detect="+ doubleToString(e.rng_detect, 1);
  s += ",rng_active="+ doubleToString(e.rng_active, 1);
  if((e.t_active > 0) && (e.t_detect > 0))
    s += ",reaction=" + doubleToString(e.t_active - e.t_detect, 2);
  if((e.t_detect > 0) && (e.t_placed > 0))
    s += ",pipeline=" + doubleToString(e.t_detect - e.t_placed, 2);
  if((e.t_end > 0) && (e.t_detect > 0))
    s += ",duration=" + doubleToString(e.t_end - e.t_detect, 2);
  s += ",dur_active="  + doubleToString(e.dur_active, 2);
  s += ",spans="       + uintToString(e.active_spans);
  s += ",mode="        + (e.mode == "" ? "none" : e.mode);
  s += ",modes="       + (e.mode_history == "" ? "none" : e.mode_history);
  s += ",apwt_max="    + doubleToString(e.apwt_max, 3);
  s += ",deflection="  + doubleToString(e.deflection_max, 1);
  s += ",deflection_valid=" + string(e.deflection_valid ? "true" : "false");
  s += ",spd_min="     + doubleToString(e.spd_min, 2);
  if(e.ucd_cpa >= 0)
    s += ",ucd_cpa="   + doubleToString(e.ucd_cpa, 2);
  if(e.post_mortem != "")
    s += ",post_mortem=" + e.post_mortem;
  s += ",end_reason="  + (e.end_reason == "" ? "none" : e.end_reason);
  return(s);
}

//---------------------------------------------------------
// Procedure: encounterJson()

string ColavEval::encounterJson(const Encounter& e) const
{
  string s = "{";
  s += "\"id\":\""      + jsonSafe(e.cn) + "-" + uintToString(e.seq) + "\",";
  s += "\"cn\":\""      + jsonSafe(e.cn) + "\",";
  s += "\"mmsi\":"      + uintToString(e.mmsi) + ",";
  s += "\"test\":"      + string(e.is_test ? "true" : "false") + ",";
  s += "\"state\":\""   + e.state + "\",";
  // Is the behaviour influencing the helm right now, as opposed to merely
  // holding an open encounter? This is what the COLREGS box on the chart is
  // shown by: the operator is being told what the boat is doing, not what it
  // is watching.
  s += "\"active\":"    + string((e.active_since > 0) ? "true" : "false") + ",";
  s += "\"verdict\":\"" + (e.verdict == "" ? "pending" : e.verdict) + "\",";
  s += "\"range\":"     + doubleToString(e.range, 1) + ",";
  s += "\"minRange\":"  + doubleToString(e.min_range, 2) + ",";
  s += "\"cpaPred\":"   + doubleToString(e.cpa_pred, 2) + ",";
  s += "\"cpaLive\":"   + doubleToString(e.cpa_live, 2) + ",";
  s += "\"tcpa\":"      + doubleToString(e.tcpa_live, 1) + ",";
  s += "\"rngDetect\":" + doubleToString(e.rng_detect, 1) + ",";
  s += "\"rngActive\":" + doubleToString(e.rng_active, 1) + ",";
  s += "\"reaction\":"  + doubleToString(((e.t_active > 0) && (e.t_detect > 0))
                                         ? (e.t_active - e.t_detect) : NA, 2) + ",";
  s += "\"pipeline\":"  + doubleToString(((e.t_detect > 0) && (e.t_placed > 0))
                                         ? (e.t_detect - e.t_placed) : NA, 2) + ",";
  s += "\"duration\":"  + doubleToString(((e.t_detect > 0))
                                         ? ((e.t_end > 0 ? e.t_end : MOOSTime()) - e.t_detect)
                                         : NA, 1) + ",";
  s += "\"durActive\":" + doubleToString(e.dur_active, 1) + ",";
  s += "\"spans\":"     + uintToString(e.active_spans) + ",";
  s += "\"mode\":\""    + jsonSafe(e.mode == "" ? "none" : e.mode) + "\",";
  s += "\"modes\":\""   + jsonSafe(e.mode_history == "" ? "none" : e.mode_history) + "\",";
  s += "\"apwtMax\":"   + doubleToString(e.apwt_max, 3) + ",";
  s += "\"deflection\":" + doubleToString(e.deflection_max, 1) + ",";
  s += "\"desDeflection\":" + doubleToString(e.des_deflection_max, 1) + ",";
  s += "\"deflectionValid\":" + string(e.deflection_valid ? "true" : "false") + ",";
  s += "\"spdMin\":"    + doubleToString(e.spd_min, 2) + ",";
  s += "\"ucdCpa\":"    + doubleToString(e.ucd_cpa, 2) + ",";
  s += "\"postMortem\":\"" + jsonSafe(e.post_mortem) + "\",";
  s += "\"endReason\":\""  + jsonSafe(e.end_reason) + "\"";
  s += "}";
  return(s);
}

//---------------------------------------------------------
// Procedure: postMetrics()
//   One latest-value string holding every record the UI should show. A browser
//   that connects late or reloads gets the whole picture from the next post,
//   with nothing to replay - the same reason PID_GAINS is shaped this way.

void ColavEval::postMetrics()
{
  string json = "[";
  for(int i=(int)m_encounters.size()-1, k=0; i>=0; i--, k++) {
    if(k > 0)
      json += ",";
    json += encounterJson(m_encounters[i]);
  }
  json += "]";

  if(json == m_last_metrics)
    return;
  m_last_metrics = json;
  Notify("COLAV_METRICS", json);
}

//---------------------------------------------------------
// Procedure: buildReport()

bool ColavEval::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "             pColavEval                     " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Test mode  : " << (m_test_mode ? "ON" : "off")
                            << "   (COLAV_TEST)" << endl;
  m_msgs << "Mission    : " << (m_waypoint_active ? "WAYPOINT running"
            : "WAYPOINT false - avdcol_ is idle, nothing will be measured") << endl;
  m_msgs << "Ownship    : " << (m_os_known
            ? (doubleToString(m_osx,1) + ", " + doubleToString(m_osy,1) +
               "  hdg " + doubleToString(m_osh,1))
            : string("(waiting for NAV_X/NAV_Y)")) << endl;
  m_msgs << "Thresholds : collision <= " << doubleToString(m_collision_range,0)
         << " m, near miss <= " << doubleToString(m_near_miss_range,0) << " m" << endl;
  m_msgs << "Contacts   : " << uintToString((unsigned int) m_contacts.size()) << endl;
  m_msgs << endl;

  ACTable actab(8);
  actab << "Contact | State | Mode | Rng | MinRng | React | Defl | Verdict";
  actab.addHeaderLines();
  for(int i=(int)m_encounters.size()-1; i>=0; i--) {
    const Encounter& e = m_encounters[i];
    string react = "--";
    if((e.t_active > 0) && (e.t_detect > 0))
      react = doubleToString(e.t_active - e.t_detect, 1) + "s";
    actab << e.cn << e.state << (e.mode == "" ? "-" : e.mode)
          << (e.range   >= 0 ? doubleToString(e.range, 0)     : "--")
          << (e.min_range >= 0 ? doubleToString(e.min_range, 1) : "--")
          << react
          << (doubleToString(e.deflection_max, 0) + (e.deflection_valid ? "" : "?"))
          << (e.verdict == "" ? "-" : e.verdict);
  }
  m_msgs << actab.getFormattedString();

  return(true);
}
