/************************************************************/
/*    FILE: EngineWatch.cpp                                 */
/************************************************************/

#include <chrono>
#include <cstdlib>
#include <sstream>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include "MBUtils.h"
#include "ACTable.h"
#include "EngineWatch.h"

using namespace std;

static size_t writeBody(void* data, size_t size, size_t n, void* out)
{
  static_cast<string*>(out)->append(static_cast<char*>(data), size * n);
  return size * n;
}

// value of a Signal K leaf ({"value": x, "timestamp": ...}) if numeric
static bool skNumber(const Json::Value& node, double& v, string* ts = 0)
{
  if(!node.isObject() || !node["value"].isNumeric())
    return false;
  v = node["value"].asDouble();
  if(ts)
    *ts = node["timestamp"].asString();
  return true;
}

EngineWatch::EngineWatch()
{
  m_source = "moos";
  m_sk_url = "http://127.0.0.1:3000/signalk/v1/api/vessels/self";
  m_stop = false;
  m_sample = Sample();
  m_sample.ok = false;
  m_last_state_post = -1;
  m_alerts_raised = 0;
  m_sk_status = "-";
}

EngineWatch::~EngineWatch()
{
  m_stop = true;
  if(m_thread.joinable())
    m_thread.join();
}

bool EngineWatch::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  double now = MOOSTime();
  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if(key == "IVPHELM_STATE")
      m_helm = msg.GetString();
    else if(m_source != "moos" || !msg.IsDouble()) {
      if(key != "APPCAST_REQ")
        reportRunWarning("Unhandled Mail: " + key);
    }
    else if(key == "ENGINE_SPEED")           m_model.rpm(now, msg.GetDouble());
    else if(key == "ENGINE_COOLANT_TEMP")    m_model.coolantTemp(msg.GetDouble());
    else if(key == "ENGINE_BATTERY_VOLTAGE") m_model.batteryVoltage(msg.GetDouble());
    else if(key == "ENGINE_FUEL_LEVEL")      m_model.fuelLevel(msg.GetDouble());
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }
  return(true);
}

bool EngineWatch::OnConnectToServer()
{
  registerVariables();
  return(true);
}

// Latest Signal K sample into the model. A reading counts as new only when
// the server's revolutions timestamp moves, so a frozen server goes stale.
void EngineWatch::takeSignalK(double now)
{
  Sample s;
  {
    lock_guard<mutex> lock(m_mutex);
    s = m_sample;
  }
  m_sk_status = s.ok ? "ok" : (s.error.empty() ? "waiting" : s.error);
  if(!s.ok)
    return;
  if(!s.engine_id.empty()) {
    string name = s.engine_id;
    name[0] = toupper(name[0]);
    m_model.setEngineName(name + " engine");
  }
  if(s.has_rpm && s.rpm_ts != m_last_rpm_ts) {
    m_last_rpm_ts = s.rpm_ts;
    m_model.rpm(now, s.rpm);
  }
  if(s.has_temp) m_model.coolantTemp(s.temp);
  if(s.has_alt)  m_model.alternatorVoltage(s.alt);
  if(s.has_batt) m_model.batteryVoltage(s.batt);
  if(s.has_fuel) m_model.fuelLevel(s.fuel);
}

bool EngineWatch::Iterate()
{
  AppCastingMOOSApp::Iterate();

  double now = MOOSTime();
  if(m_source == "signalk")
    takeSignalK(now);
  m_model.setMission(m_helm == "DRIVE");

  vector<EngineWatchModel::Change> changes = m_model.update(now);
  for(size_t i = 0; i < changes.size(); i++) {
    const EngineWatchModel::Change& c = changes[i];
    string level = EngineWatchModel::levelName(c.level);
    Notify("ENGINE_ALERT", "key=" + c.key + ",level=" + level + ",msg=" + c.msg);
    // one run warning per key; the exact text is kept to retract it
    if(m_warnings.count(c.key)) {
      retractRunWarning(m_warnings[c.key]);
      m_warnings.erase(c.key);
    }
    if(c.level != EngineWatchModel::NONE) {
      m_warnings[c.key] = "ENGINE " + level + ": " + c.msg;
      reportRunWarning(m_warnings[c.key]);
      reportEvent("ENGINE_ALERT " + level + ": " + c.msg);
      m_alerts_raised++;
    }
    else
      reportEvent("ENGINE_ALERT cleared: " + c.key);
  }

  string state = EngineWatchModel::runStateName(m_model.runState());
  if(state != m_posted_state || now - m_last_state_post >= 5.0) {
    if(state != m_posted_state)
      reportEvent("ENGINE_RUN_STATE " + state);
    Notify("ENGINE_RUN_STATE", state);
    m_posted_state = state;
    m_last_state_post = now;
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

// Worker thread: GET vessels/self about once a second. A short timeout keeps
// a hung server from piling up requests; Iterate never waits on it.
void EngineWatch::pollLoop()
{
  CURL* curl = curl_easy_init();
  struct curl_slist* headers = 0;
  if(!m_sk_token.empty())
    headers = curl_slist_append(headers, ("Authorization: Bearer " + m_sk_token).c_str());

  while(!m_stop) {
    Sample s = Sample();
    s.ok = false;
    string body;
    if(!curl)
      s.error = "curl init failed";
    else {
      curl_easy_setopt(curl, CURLOPT_URL, m_sk_url.c_str());
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeBody);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &body);
      curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 900L);
      curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
      if(headers)
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
      CURLcode res = curl_easy_perform(curl);
      long code = 0;
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
      if(res != CURLE_OK)
        s.error = curl_easy_strerror(res);
      else if(code != 200)
        s.error = "HTTP " + to_string(code);
    }

    Json::Value root;
    if(s.error.empty() && curl) {
      Json::CharReaderBuilder rb;
      string errs;
      istringstream in(body);
      if(!Json::parseFromStream(rb, in, &root, &errs))
        s.error = "bad JSON";
    }
    if(s.error.empty()) {
      const Json::Value& prop = root["propulsion"];
      string id = m_engine_id;
      if(id.empty() && prop.isObject() && !prop.getMemberNames().empty())
        id = prop.getMemberNames()[0];
      if(id.empty() || !prop.isMember(id))
        s.error = "no propulsion" + (id.empty() ? string("") : "." + id);
      else {
        const Json::Value& e = prop[id];
        s.ok = true;
        s.engine_id = id;
        s.has_rpm = skNumber(e["revolutions"], s.rpm, &s.rpm_ts);
        s.rpm *= 60;   // Hz -> rpm
        s.has_temp = skNumber(e["coolantTemperature"], s.temp) || skNumber(e["temperature"], s.temp);
        s.temp -= 273.15;   // K -> C
        s.has_alt = skNumber(e["alternatorVoltage"], s.alt);
        s.has_batt = skNumber(root["electrical"]["batteries"]["House"]["voltage"], s.batt);
        s.has_fuel = skNumber(root["tanks"]["fuel"]["0"]["currentLevel"], s.fuel);
        s.fuel *= 100;      // ratio -> %
      }
    }
    {
      lock_guard<mutex> lock(m_mutex);
      m_sample = s;
    }
    for(int i = 0; i < 10 && !m_stop; i++)
      this_thread::sleep_for(chrono::milliseconds(100));
  }
  if(headers)
    curl_slist_free_all(headers);
  if(curl)
    curl_easy_cleanup(curl);
}

bool EngineWatch::OnStartUp()
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
    if(param == "engine_source") {
      string v = tolower(value);
      handled = (v == "moos" || v == "signalk");
      if(handled)
        m_source = v;
    }
    // host:port only, as iWebSocket / grafana_bridge: // starts a MOOS comment
    else if(param == "signalk_http" && !value.empty() && value.find('/') == string::npos) {
      m_sk_url = "http://" + value + "/signalk/v1/api/vessels/self";
      handled = true;
    }
    else if(param == "signalk_token") {
      m_sk_token = value;
      handled = true;
    }
    else if(param == "engine_id") {
      m_engine_id = value;
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  if(m_source == "signalk") {
    curl_global_init(CURL_GLOBAL_DEFAULT);
    m_thread = thread(&EngineWatch::pollLoop, this);
  }

  registerVariables();
  return(true);
}

void EngineWatch::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("IVPHELM_STATE", 0);
  if(m_source == "moos") {
    Register("ENGINE_SPEED", 0);
    Register("ENGINE_COOLANT_TEMP", 0);
    Register("ENGINE_BATTERY_VOLTAGE", 0);
    Register("ENGINE_FUEL_LEVEL", 0);
  }
}

bool EngineWatch::buildReport()
{
  double now = MOOSTime();
  double age = m_model.secsSinceData(now);
  m_msgs << "Source:    " << m_source;
  if(m_source == "signalk")
    m_msgs << " (" << m_sk_url << ": " << m_sk_status << ")";
  m_msgs << endl;
  m_msgs << "Helm:      " << (m_helm.empty() ? "?" : m_helm)
         << (m_model.mission() ? "  (mission)" : "") << endl;
  m_msgs << endl;

  ACTable actab(2);
  actab << "Item | Value";
  actab.addHeaderLines();
  actab << "engine" << m_model.engineName();
  actab << "ENGINE_RUN_STATE" << EngineWatchModel::runStateName(m_model.runState());
  actab << "rpm" << (age < 0 ? string("-") : doubleToString(m_model.lastRpm(), 0));
  actab << "temp (C)" << (m_model.hasTemp() ? doubleToString(m_model.lastTemp(), 1) : string("-"));
  actab << "data age (s)" << (age < 0 ? string("never") : doubleToString(age, 1));
  actab << "alerts raised" << uintToString(m_alerts_raised);
  m_msgs << actab.getFormattedString() << endl << endl;

  m_msgs << "Active alerts:" << endl;
  const map<string, EngineWatchModel::Alert>& a = m_model.active();
  if(a.empty())
    m_msgs << "  none" << endl;
  map<string, EngineWatchModel::Alert>::const_iterator q;
  for(q = a.begin(); q != a.end(); q++)
    m_msgs << "  " << EngineWatchModel::levelName(q->second.level) << "  "
           << q->first << ": " << q->second.msg << endl;
  return(true);
}
