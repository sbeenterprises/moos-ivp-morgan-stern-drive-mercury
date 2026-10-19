/************************************************************/
/*    NAME: D Lima                                          */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: WebSocket.cpp                                   */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <jsoncpp/json/json.h>     // Biblioteca jsoncpp
#include <sstream>
#include <iostream>
#include <algorithm>
#include "MBUtils.h"
#include "ACTable.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "../MOOSGeodesy.cpp"
#include "WebSocket.h"

using namespace std;

#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXUserAgent.h>
#include <ixwebsocket/IXWebSocketServer.h>
#include <iostream>
#include <iomanip>
#include <stdexcept>

//Variáveis para utilizar com o WebSocket
double nav_lat = 0;
double nav_long = 0;
double nav_speed = 0;
double nav_heading = 0;
double desired_heading = 0;
double desired_speed = 0;
double desired_rudder = 0;
double desired_thrust = 0;
double value_setpoint = 0;
double wpt_index = 0; // Index of the current waypoint
double wpt_pct_complete = 0; // Percentage of route completed
double engine_accelerator_pedal_position = 0;
double engine_percent_load = 0;
double engine_speed = 0;
double engine_percent_torque = 0;
double engine_active_dtcs = 0;
double engine_coolant_temp = 0;
double engine_oil_temp = 0;
double engine_oil_pressure = 0;
double engine_fuel_rate = 0;
double engine_intake_manifold_pressure = 0;
double engine_battery_voltage = 0;
std::string ivphelm_state;
std::string ivphelm_allstop;
std::string waypoint;
std::string wpt_stat;
std::string view_point;
std::string id_route;
std::string wpt_radii;
bool repeat_mission = false;
std::string base_coords;
bool can_send = false; // Variable used to control CAN message flux
bool region_active = false; // Variable used to control region activation
bool service_mode = false; // PID calibration mode driven from the UI
std::string pid_gains; // Gains planchaPID is actually applying (see PID_GAINS)


// Trail (follow-a-contact) state, mirrored back to the UI so it can show what
// the helm is really doing rather than what the UI asked for.
bool trailing = false;
std::string trail_updates;

// Collision avoidance test state. COLAV_TEST is the request echo; the ack comes
// from pColavEval and is what actually proves an evaluator is running, so the
// UI toggle follows the ack rather than its own optimism.
bool colav_test = false;
bool colav_test_ack = false;
std::string colav_metrics;    // JSON array of encounters, latest value only
std::string traffic_spawned;  // last pTrafficSim ack, ok=true/false
std::string traffic_targets;  // runtime-spawned contacts currently placed

// Engagement distances of the collision avoidance behaviour (BHV_AvdColregsV22,
// the avdcol_ block in alpha.bhv). Seeded here with the mission values and
// overridable from the iWebSocket config block, then owned by this app: the UI
// edits them, applyColavDist() pushes them into the behaviour, and every
// spawned instance is brought up to date the same way. Metres.
double colav_outer     = 300;   // pwt_outer_dist
double colav_inner     = 150;   // pwt_inner_dist
double colav_completed = 1500;  // completed_dist
double colav_min_cpa   = 100;   // min_util_cpa_dist
double colav_max_cpa   = 300;   // max_util_cpa_dist
std::string colav_dist;         // the set as last applied, echoed to the UI

// A behaviour instance is built from the template, so it comes up with the
// values in alpha.bhv rather than the ones in force. Its spawnxflag lands here
// and the set is pushed again on the next tick.
bool colav_respawn_seen = false;

// NODE_REPORT contact variables
std::string contact_name = "";
double contact_lat = 0;
double contact_lon = 0;
double contact_spd = 0;
double contact_hdg = 0;


//---------------------------------------------------------
// Constructor()

WebSocket::WebSocket()
{
  msg_recebida = "";
  m_connection_timeout = 60.0; // Default 60 seconds timeout
  m_last_connection_check = 0.0;
  m_heartbeat_interval = 30.0; // Send heartbeat every 30 seconds (less than frontend's 60s timeout)
  m_last_heartbeat_time = 0.0;
  m_last_link_rx = -1e9;
  m_link_timeout = 3.0;
  m_origin_ok = false;
  m_lat_origin = 0;
  m_lon_origin = 0;
  m_link_ok_posted = false;
  m_last_link_post = 0.0;

  const char* can_vars[] = {"CAN_STATUS", "CAN_CONTROL_OWNER", "CAN_LOST_LINK",
                            "CAN_CONFLICT", "CAN_TX_RUDDER_PCT", "CAN_TX_THRUST_PCT",
                            "CAN_TX_GEAR", "CAN_TRIM_STATE", "CAN_RUDDER_MISMATCH", "CAN_PROFILE",
                            "CAN_RUDDER_ACTUAL_PCT", "CAN_HELM_HAS_STEERING",
                            "CAN_ENGINE_GEAR", "CAN_ENGINE_THROTTLE_PCT", "CAN_FUEL_PCT"};
  for (size_t i = 0; i < sizeof(can_vars) / sizeof(can_vars[0]); i++)
    m_can_vars.insert(can_vars[i]);

  initializeMessageHandlers();
}

// Implement the new methods for multiple clients
void WebSocket::AddWebSocket(ix::WebSocket* ws)
{
  if (ws != nullptr) {
    m_webSockets.push_back(ws);
    m_lastMessageTime[ws] = MOOSTime(); // Initialize with current time
    m_last_link_rx = MOOSTime();
    
    // Initialize heartbeat time when first client connects
    if (m_webSockets.size() == 1) {
      m_last_heartbeat_time = MOOSTime();
      reportEvent("First client connected - heartbeat timer initialized");
    }
    
    reportEvent("New WebSocket client connected. Total clients: " + std::to_string(m_webSockets.size()));
  }
}

void WebSocket::RemoveWebSocket(ix::WebSocket* ws)
{
  auto it = std::find(m_webSockets.begin(), m_webSockets.end(), ws);
  if (it != m_webSockets.end()) {
    m_webSockets.erase(it);
    m_lastMessageTime.erase(ws); // Remove timestamp tracking
    reportEvent("WebSocket client disconnected. Total clients: " + std::to_string(m_webSockets.size()));
  }
}

bool WebSocket::HasConnectedClients() const
{
  return !m_webSockets.empty();
}

size_t WebSocket::GetConnectedClientCount() const
{
  return m_webSockets.size();
}

void WebSocket::UpdateConnectionTime(ix::WebSocket* ws)
{
  if (ws != nullptr) {
    m_lastMessageTime[ws] = MOOSTime();
    m_last_link_rx = MOOSTime();
  }
}

void WebSocket::CheckStaleConnections()
{
  double current_time = MOOSTime();
  std::vector<ix::WebSocket*> stale_connections;
  
  // Find stale connections
  for (auto& pair : m_lastMessageTime) {
    double time_since_last_msg = current_time - pair.second;
    if (time_since_last_msg > m_connection_timeout) {
      stale_connections.push_back(pair.first);
      reportRunWarning("Detected stale connection (" + 
                      std::to_string(time_since_last_msg) + 
                      "s since last message). Timeout: " + 
                      std::to_string(m_connection_timeout) + "s");
    }
  }
  
  // Remove stale connections
  for (auto* ws : stale_connections) {
    RemoveWebSocket(ws);
  }
}

void WebSocket::SendMessage(const std::string& message, bool binary)
{
  if (m_webSockets.empty())
  {
    reportRunWarning("Attempted to send message but no WebSocket clients connected");
    return;
  }

  // Broadcast message to all connected clients
  for (auto* ws : m_webSockets)
  {
    if (ws != nullptr)
    {
      ws->send(message, binary);
    }
  }
}

// Wrapper methods implementation for the protected Notify methods
bool WebSocket::NotifyMOOS(const std::string& key, const std::string& value)
{
  return Notify(key, value);
}

bool WebSocket::NotifyMOOS(const std::string& key, double value)
{
  return Notify(key, value);
}

bool WebSocket::NotifyMOOS(const std::string& key, bool value)
{
  return Notify(key, static_cast<double>(value));
}

//---------------------------------------------------------
// Destructor

WebSocket::~WebSocket()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool WebSocket::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p = NewMail.begin(); p != NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if(key == "NAV_HEADING")
      nav_heading = msg.GetDouble();
    else if(key == "DESIRED_HEADING")
      desired_heading = msg.GetDouble();
    else if(key == "DESIRED_SPEED")
      desired_speed = msg.GetDouble();
    else if(key == "DESIRED_RUDDER")
      desired_rudder = msg.GetDouble();
    else if(key == "DESIRED_THRUST")
      desired_thrust = msg.GetDouble();
    else if(key == "IVPHELM_STATE")
      ivphelm_state = msg.GetString();
    else if(key == "WPT_INDEX")
      wpt_index = msg.GetDouble();
    else if(key == "WPT_PCT_COMPLETE")
      wpt_pct_complete = msg.GetDouble();
    else if(key == "IVPHELM_ALLSTOP")
      ivphelm_allstop = msg.GetString();
    else if(key == "WAYPOINT")
      waypoint = msg.GetString();
    else if(key == "WPT_STAT")
      wpt_stat = msg.GetString();
    else if(key == "WPT_RADII")
      wpt_radii = msg.GetString();
    else if(key == "ENGINE_ACCELERATOR_PEDAL_POSITION")
      engine_accelerator_pedal_position = msg.GetDouble();
    else if(key == "ENGINE_PERCENT_LOAD")
      engine_percent_load = msg.GetDouble();
    else if(key == "ENGINE_SPEED")
      engine_speed = msg.GetDouble();
    else if(key == "ENGINE_PERCENT_TORQUE")
      engine_percent_torque = msg.GetDouble();
    else if(key == "ENGINE_ACTIVE_DTCS")
      engine_active_dtcs = msg.GetDouble();
    else if(key == "ENGINE_COOLANT_TEMP")
      engine_coolant_temp = msg.GetDouble();
    else if(key == "ENGINE_OIL_TEMP")
      engine_oil_temp = msg.GetDouble();
    else if(key == "ENGINE_OIL_PRESSURE")
      engine_oil_pressure = msg.GetDouble();
    else if(key == "ENGINE_FUEL_RATE")
      engine_fuel_rate = msg.GetDouble();
    else if(key == "ENGINE_INTAKE_MANIFOLD_PRESSURE")
      engine_intake_manifold_pressure = msg.GetDouble();
    else if(key == "ENGINE_BATTERY_VOLTAGE")
      engine_battery_voltage = msg.GetDouble();
    else if(key == "VIEW_POINT")
      view_point = msg.GetString();
    else if(key == "BASE_COORDS")
      base_coords = msg.GetString();
    else if(key == "CAN_SEND")
      can_send = (msg.GetString() == "true");
    else if(m_can_vars.count(key)) {
      if(msg.IsDouble())
        m_can_num[key] = msg.GetDouble();
      else
        m_can_str[key] = msg.GetString();
    }
    else if(key == "REGION")
      region_active = (msg.GetString() == "true");
    else if(key == "SERVICE_MODE")
      service_mode = (msg.GetString() == "true");
    else if(key == "PID_GAINS")
      pid_gains = msg.GetString();
    else if(key == "TRAILING")
      trailing = (msg.GetString() == "true");
    else if(key == "TRAIL_UPDATES")
      trail_updates = msg.GetString();
    else if(key == "COLAV_TEST")
      colav_test = (msg.GetString() == "true");
    else if(key == "COLAV_TEST_ACK")
      colav_test_ack = (msg.GetString() == "true");
    else if(key == "COLAV_METRICS")
      colav_metrics = msg.GetString();
    else if(key == "TRAFFIC_SPAWNED")
      traffic_spawned = msg.GetString();
    else if(key == "TRAFFIC_TARGETS")
      traffic_targets = msg.GetString();
    else if(key == "COLAV_BHV_SPAWN")
      colav_respawn_seen = true;
    else if(key == "NODE_REPORT") {
      // Parse NODE_REPORT to extract NAME, X, Y, SPD, HDG
      std::string node_report = msg.GetString();

      std::string name = "";
      double x = 0, y = 0;
      double spd = 0, hdg = 0;

      // Parse the comma-separated key=value pairs
      std::istringstream iss(node_report);
      std::string token;

      while (std::getline(iss, token, ',')) {
        size_t eq_pos = token.find('=');
        if (eq_pos != std::string::npos) {
          std::string key_part = token.substr(0, eq_pos);
          std::string val_part = token.substr(eq_pos + 1);

          if (key_part == "NAME") {
            name = val_part;
          } else if (key_part == "X") {
            x = std::stod(val_part);
          } else if (key_part == "Y") {
            y = std::stod(val_part);
          } else if (key_part == "SPD") {
            spd = std::stod(val_part);
          } else if (key_part == "HDG") {
            hdg = std::stod(val_part);
          }
        }
      }

      // Convert local X,Y to lat/lon, using the same projection as
      // pMarineViewer and pNodeReporter (see USE_UTM in the top-level
      // CMakeLists). This used the flat-earth grid plus a hand-tuned
      // "y += 100" that cancelled most - not all - of the resulting error;
      // the projection is the actual fix, so the fudge is gone.
      // Origin: the mission's LatOrigin / LongOrigin (OnStartUp).
      if (!m_origin_ok)
        continue;

      double lat, lon;
#ifdef USE_UTM
      m_geodesy.UTM2LatLong(x, y, lat, lon);
#else
      m_geodesy.LocalGrid2LatLong(x, y, lat, lon);
#endif

      // Store the values
      contact_name = name;
      contact_lat = lat;
      contact_lon = lon;
      contact_spd = spd;
      contact_hdg = hdg;
    }
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool WebSocket::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: QueueMessage()
//            called from the websocket thread for every message the UI sends

void WebSocket::QueueMessage(const std::string& message)
{
  std::lock_guard<std::mutex> lock(m_inbox_mutex);

  // Bound the queue so a stalled MOOS thread cannot grow it without limit.
  // Dropping the oldest keeps the most recent commands, which are the ones
  // the operator is waiting on.
  while (m_inbox.size() >= 512)
    m_inbox.pop_front();

  m_inbox.push_back(message);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool WebSocket::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  double current_time = MOOSTime();
  
  // Check for stale connections every 10 seconds
  if (current_time - m_last_connection_check > 10.0) {
    CheckStaleConnections();
    m_last_connection_check = current_time;
  }
  
  // Send heartbeat message to keep connection alive
  if (HasConnectedClients() && (current_time - m_last_heartbeat_time > m_heartbeat_interval)) {
    SendMessage("HEARTBEAT=" + std::to_string(current_time));
    m_last_heartbeat_time = current_time;
  }
  
  // Link to SS UI: at least one client connected and something heard from
  // it within link_timeout. iCAN uses this for the lost-link failsafe in
  // manual control; it is posted on change and once a second so iCAN can
  // also tell when iWebSocket itself has stopped.
  bool link_ok = HasConnectedClients() &&
                 (current_time - m_last_link_rx <= m_link_timeout);
  if (link_ok != m_link_ok_posted || current_time - m_last_link_post >= 1.0) {
    if (link_ok != m_link_ok_posted)
      reportEvent(std::string("SS UI link ") + (link_ok ? "up" : "LOST"));
    Notify("UI_LINK_OK", link_ok ? "true" : "false");
    m_link_ok_posted = link_ok;
    m_last_link_post = current_time;
  }

  // Aqui você pode incluir outras operações se necessário

  //Analysis the variables coming from Freeboard
  // Take everything the websocket thread queued since the last tick. The UI
  // sends messages in bursts - enabling service mode pushes SERVICE_MODE
  // followed immediately by the six PID gains - and every one of them has to be
  // dispatched, not just whichever arrived last.
  std::deque<std::string> inbox;
  {
    std::lock_guard<std::mutex> lock(m_inbox_mutex);
    inbox.swap(m_inbox);
  }

  for (const std::string& message : inbox) {
    msg_recebida = message; // keep the appcast report showing the last message

    //Split in the equal sign
    size_t equalsPos = message.find('=');
    if (equalsPos == std::string::npos)
      continue;

    //take the two sides
    std::string key = message.substr(0, equalsPos);
    std::string value = message.substr(equalsPos + 1);

    std::cout << "Key: " << key << "\n";
    std::cout << "Value: " << value << "\n";

    // State machine approach: O(log n) lookup instead of O(n) linear search
    auto handler = m_messageHandlers.find(key);
    if (handler != m_messageHandlers.end()) {
      (this->*(handler->second))(value);
    }
  }

  // A fresh avdcol_ instance carries the template's distances, not the ones
  // the operator set. Push them again so a contact that spawned a behaviour is
  // avoided with the same envelope as the one before it.
  if (colav_respawn_seen) {
    colav_respawn_seen = false;
    applyColavDist();
  }

  // If you have pending messages to send
  //Send messages to the websocket in FreeboardSK
  //SendMessage("NAV_LAT=50.0"); // Exemplo de envio de mensagem
  //SendMessage("NAV_HEADING=" + std::to_string(nav_heading)); // Sends atual heading
  //For some reason, is just sending from this on
  //I put the value_setpoint to facilitate the calibration
  //SendMessage("DESIRED_HEADING=" + std::to_string(value_setpoint)+"/NAV_HEADING="+std::to_string(nav_heading)); // Sends desired heading
  // Build consolidated JSON message with all telemetry data
  Json::Value root;
  root["IVPHELM_STATE"] = ivphelm_state;
  root["WPT_INDEX"] = wpt_index;
  root["IVPHELM_ALLSTOP"] = ivphelm_allstop;
  root["WAYPOINT"] = waypoint;
  root["WPT_STAT"] = wpt_stat;
  root["VIEW_POINT"] = view_point;
  root["ID_ROUTE"] = id_route;
  root["REPEAT_MISSION"] = repeat_mission;
  root["DESIRED_SPEED"] = desired_speed;
  root["BASE_COORDS"] = base_coords;
  root["CAN_SEND"] = can_send;
  root["REGION"] = region_active;
  root["DESIRED_RUDDER"] = desired_rudder;
  root["DESIRED_THRUST"] = desired_thrust;
  root["ENGINE_ACCELERATOR_PEDAL_POSITION"] = engine_accelerator_pedal_position;
  root["ENGINE_PERCENT_LOAD"] = engine_percent_load;
  root["ENGINE_SPEED"] = engine_speed;
  root["ENGINE_PERCENT_TORQUE"] = engine_percent_torque;
  root["ENGINE_ACTIVE_DTCS"] = engine_active_dtcs;
  root["ENGINE_COOLANT_TEMP"] = engine_coolant_temp;
  root["ENGINE_OIL_TEMP"] = engine_oil_temp;
  root["ENGINE_OIL_PRESSURE"] = engine_oil_pressure;
  root["ENGINE_FUEL_RATE"] = engine_fuel_rate;
  root["ENGINE_INTAKE_MANIFOLD_PRESSURE"] = engine_intake_manifold_pressure;
  root["ENGINE_BATTERY_VOLTAGE"] = engine_battery_voltage;
  root["WPT_PCT_COMPLETE"] = wpt_pct_complete;
  root["WPT_RADII"] = wpt_radii;
  root["SERVICE_MODE"] = service_mode;
  root["PID_GAINS"] = pid_gains;
  root["TRAILING"] = trailing;
  root["TRAIL_UPDATES"] = trail_updates;
  root["COLAV_TEST"] = colav_test;
  root["COLAV_TEST_ACK"] = colav_test_ack;
  root["COLAV_METRICS"] = colav_metrics;
  root["TRAFFIC_SPAWNED"] = traffic_spawned;
  root["TRAFFIC_TARGETS"] = traffic_targets;
  root["COLAV_DIST"] = colav_dist;

  // iCAN: what is really being sent, who owns the drive, feedback
  for (std::map<std::string, double>::iterator v = m_can_num.begin(); v != m_can_num.end(); v++)
    root[v->first] = v->second;
  for (std::map<std::string, std::string>::iterator v = m_can_str.begin(); v != m_can_str.end(); v++)
    root[v->first] = v->second;

  // Add contact info as nested object
  Json::Value contact;
  contact["NAME"] = contact_name;
  contact["LAT"] = contact_lat;
  contact["LON"] = contact_lon;
  contact["SPD"] = contact_spd;
  contact["HDG"] = contact_hdg;
  root["CONTACT_INFO"] = contact;

  // Send single JSON message (compact format for network efficiency)
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  SendMessage(Json::writeString(writer, root));

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool WebSocket::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p = sParams.begin(); p != sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    // Startup state of the collision avoidance envelope. These have to mirror
    // the avdcol_ block in the .bhv file: until the operator touches the panel
    // the behaviour is running on its own configuration, and what is reported
    // to the UI has to be the truth about it, not a second opinion.
    if(param == "colav_pwt_outer_dist" && isNumber(value)) {
      colav_outer = atof(value.c_str());
      handled = true;
    }
    else if(param == "colav_pwt_inner_dist" && isNumber(value)) {
      colav_inner = atof(value.c_str());
      handled = true;
    }
    else if(param == "colav_completed_dist" && isNumber(value)) {
      colav_completed = atof(value.c_str());
      handled = true;
    }
    else if(param == "colav_min_util_cpa_dist" && isNumber(value)) {
      colav_min_cpa = atof(value.c_str());
      handled = true;
    }
    else if(param == "colav_max_util_cpa_dist" && isNumber(value)) {
      colav_max_cpa = atof(value.c_str());
      handled = true;
    }
    else if(param == "link_timeout" && isNumber(value) && atof(value.c_str()) > 0) {
      m_link_timeout = atof(value.c_str());
      handled = true;
    }
    else if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Not applied to the behaviour here - at this point it is only what the
  // .bhv is expected to hold. It gives the UI and the log a starting value;
  // the first edit from the panel is what actually pushes anything.
  std::ostringstream echo;
  echo << std::fixed << std::setprecision(1)
       << "pwt_outer_dist=" << colav_outer
       << ",pwt_inner_dist=" << colav_inner
       << ",completed_dist=" << colav_completed
       << ",min_util_cpa_dist=" << colav_min_cpa
       << ",max_util_cpa_dist=" << colav_max_cpa;
  colav_dist = echo.str();

  // Geodesy origin: the mission's global LatOrigin / LongOrigin, the same
  // origin pNodeReporter and iDivisorNMEA use, so routes, regions and
  // contacts land on the boat's own grid. It used to be a Morgan City
  // literal in seven places, which would have put every route and region
  // in the wrong place at any other site.
  m_origin_ok = false;
  if (!m_MissionReader.GetValue("LatOrigin", m_lat_origin) ||
      !m_MissionReader.GetValue("LongOrigin", m_lon_origin))
    reportConfigWarning("LatOrigin / LongOrigin missing from the mission file: "
                        "route, region, return, station and target commands are refused");
  else if (!m_geodesy.Initialise(m_lat_origin, m_lon_origin))
    reportConfigWarning("Geodesy initialisation failed for LatOrigin/LongOrigin");
  else
    m_origin_ok = true;

  // Registra variáveis no MOOS, se necessário
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void WebSocket::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Exemplo: Register("FOOBAR", 0);
  Register("NAV_LAT", 0);
  Register("DEPLOY", 0);
  Register("STOP_CAN", 0);
  Register("NAV_HEADING", 0);
  Register("DESIRED_HEADING", 0);
  Register("DESIRED_THRUST", 0);
  Register("DESIRED_RUDDER", 0);
  Register("CONSTANT_HEADING", 0);
  Register("DESIRED_SPEED", 0);
  Register("IVPHELM_STATE", 0);
  Register("WPT_INDEX", 0);
  Register("IVPHELM_ALLSTOP", 0);
  Register("WPT_STAT", 0);
  Register("ENGINE_ACCELERATOR_PEDAL_POSITION", 0);
  Register("ENGINE_PERCENT_LOAD", 0);
  Register("ENGINE_SPEED", 0);
  Register("ENGINE_PERCENT_TORQUE", 0);
  Register("ENGINE_ACTIVE_DTCS", 0);
  Register("ENGINE_COOLANT_TEMP", 0);
  Register("OIL_TEMP", 0);
  Register("ENGINE_OIL_PRESSURE", 0);
  Register("ENGINE_FUEL_RATE", 0);
  Register("ENGINE_INTAKE_MANIFOLD_PRESSURE", 0);
  Register("ENGINE_BATTERY_VOLTAGE", 0);
  Register("NODE_REPORT", 0);
  Register("ID_ROUTE", 0);
  Register("REPEAT_MISSION", 0);
  Register("BASE_COORDS", 0); //Variable used in return to home
  Register("CAN_SEND", 0); //Variable used to control CAN message flux
  Register("REGION", 0); //Variable used to control region
  Register("WAYPOINT", 0); //Variable to control waypoint behavior
  Register("WPT_PCT_COMPLETE", 0); //Route completion percentage
  Register("WPT_RADII", 0); //Waypoint radii configuration
  Register("SERVICE_MODE", 0); //PID calibration mode (manual gains)
  Register("PID_GAINS", 0); //Gains planchaPID is currently applying
  Register("TRAILING", 0); //Follow-a-contact behaviour active
  Register("TRAIL_UPDATES", 0); //Contact/range/angle the trail behaviour is using
  Register("COLAV_TEST", 0);      //Collision avoidance test mode, request echo
  Register("COLAV_TEST_ACK", 0);  //...confirmed by pColavEval
  Register("COLAV_METRICS", 0);   //Live + recent encounter records, JSON
  Register("TRAFFIC_SPAWNED", 0); //pTrafficSim ack for a placed target
  Register("TRAFFIC_TARGETS", 0); //Runtime-spawned contacts currently placed
  Register("COLAV_BHV_SPAWN", 0); //An avdcol_ instance was just spawned
  for (std::set<std::string>::iterator v = m_can_vars.begin(); v != m_can_vars.end(); v++)
    Register(*v, 0);              //iCAN status and drive feedback for SS UI

}

//---------------------------------------------------------
// Procedure: buildReport()

bool WebSocket::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "        Relatório de Debug - WebSocket      " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Servidor WebSocket configurado para:" << endl;
  m_msgs << "  Endereço: 0.0.0.0" << endl;
  m_msgs << "  Porta:    8000" << endl;
  m_msgs << "  Estado:   Running" << endl;
  m_msgs << endl;

  // Exemplo de uso do ACTable para formatar alguns parâmetros
  ACTable actab(2);
  actab << "Parâmetro | Valor";
  actab.addHeaderLines();
  actab << "Address" << "0.0.0.0";
  actab << "Port"    << "8000";
  actab << "Status"  << "Running";
  actab << "Connected Clients" << std::to_string(GetConnectedClientCount());
  actab << "Mensagem Recebida"  << msg_recebida;
  actab << "Connection Timeout" << std::to_string(m_connection_timeout) + "s";
  
  // Show connection health
  if (!m_webSockets.empty()) {
    m_msgs << endl << "Connection Health:" << endl;
    double current_time = MOOSTime();
    for (size_t i = 0; i < m_webSockets.size(); i++) {
      auto* ws = m_webSockets[i];
      double time_since_last_msg = current_time - m_lastMessageTime[ws];
      m_msgs << "  Client " << (i+1) << ": Last message " 
             << std::fixed << std::setprecision(1) << time_since_last_msg << "s ago" << endl;
    }
  }

  m_msgs << actab.getFormattedString();

  return true;
}

//---------------------------------------------------------
// State Machine Implementation
//---------------------------------------------------------

void WebSocket::initializeMessageHandlers()
{
  m_messageHandlers["CONTROLE_MANUAL"] = &WebSocket::handleControleManual;
  m_messageHandlers["AUTONOMOUS_CONTROL"] = &WebSocket::handleAutonomousControl;
  m_messageHandlers["DESIRED_HEADING"] = &WebSocket::handleDesiredHeading;
  m_messageHandlers["DESIRED_SPEED"] = &WebSocket::handleDesiredSpeed;
  m_messageHandlers["DESIRED_RUDDER"] = &WebSocket::handleDesiredRudder;
  m_messageHandlers["DESIRED_THRUST"] = &WebSocket::handleDesiredThrust;
  m_messageHandlers["DESIRED_GEAR"] = &WebSocket::handleDesiredGear;
  m_messageHandlers["AUTONOMOUS_WAYPOINTS"] = &WebSocket::handleAutonomousWaypoints;
  m_messageHandlers["AUTONOMOUS_START"] = &WebSocket::handleAutonomousStart;
  m_messageHandlers["AUTONOMOUS_STOP"] = &WebSocket::handleAutonomousStop;
  m_messageHandlers["AUTONOMOUS_RETURN_TO_BASE"] = &WebSocket::handleAutonomousReturnToBase;
  m_messageHandlers["RETURN_UPDATE"] = &WebSocket::handleReturnUpdate;
  m_messageHandlers["STOP_CAN"] = &WebSocket::handleStopCan;
  m_messageHandlers["CONSTANT_HEADING"] = &WebSocket::handleConstantHeading;
  m_messageHandlers["SETPOINT_HEADING"] = &WebSocket::handleSetpointHeading;
  m_messageHandlers["HEADING_KP"] = &WebSocket::handleHeadingKp;
  m_messageHandlers["HEADING_KI"] = &WebSocket::handleHeadingKi;
  m_messageHandlers["HEADING_KD"] = &WebSocket::handleHeadingKd;
  m_messageHandlers["SPEED_KP"] = &WebSocket::handleSpeedKp;
  m_messageHandlers["SPEED_KI"] = &WebSocket::handleSpeedKi;
  m_messageHandlers["SPEED_KD"] = &WebSocket::handleSpeedKd;
  m_messageHandlers["SERVICE_MODE"] = &WebSocket::handleServiceMode;
  m_messageHandlers["TRIM"] = &WebSocket::handleTrim;
  m_messageHandlers["STATION_KEEP_WAYPOINT"] = &WebSocket::handleStationKeepWaypoint;
  m_messageHandlers["HEARTBEAT_PING"] = &WebSocket::handleHeartbeatPing;
  m_messageHandlers["REPEAT"] = &WebSocket::handleRepeat;
  m_messageHandlers["BASE_COORDS"] = &WebSocket::handleBaseCoords;
  m_messageHandlers["CAN_SEND"] = &WebSocket::handleCanSend;
  m_messageHandlers["REGION_UPDATES"] = &WebSocket::handleRegionUpdates;
  m_messageHandlers["REGION"] = &WebSocket::handleRegion;
  m_messageHandlers["TRAIL_FOLLOW"] = &WebSocket::handleTrailFollow;
  m_messageHandlers["COLAV_TEST"] = &WebSocket::handleColavTest;
  m_messageHandlers["PUT_TARGET"] = &WebSocket::handlePutTarget;
  m_messageHandlers["CLEAR_TARGETS"] = &WebSocket::handleClearTargets;
  m_messageHandlers["COLAV_DIST"] = &WebSocket::handleColavDist;
}

//---------------------------------------------------------
// Procedure: handleTrailFollow()
//   TRAIL_FOLLOW=mmsi=366999001#range=150#angle=180
//   TRAIL_FOLLOW=off
//
//   The contact name is the MMSI with an AIS prefix - pTrafficSim names its
//   NODE_REPORTs the same way, so the UI never needs a name/MMSI lookup.

void WebSocket::handleTrailFollow(const std::string& value)
{
  if (value == "off" || value == "false") {
    Notify("TRAILING", "false");
    // Same safety pair AUTONOMOUS_STOP uses: leave the boat with no residual
    // command once the behaviour releases the helm.
    Notify("DESIRED_RUDDER", 0.0);
    Notify("DESIRED_THRUST", 0.0);
    std::cout << "[TRAIL_FOLLOW] stopped" << std::endl;
    return;
  }

  std::string mmsi;
  std::string range;
  std::string angle;

  std::stringstream ss(value);
  std::string token;
  while (std::getline(ss, token, '#')) {
    size_t eq = token.find('=');
    if (eq == std::string::npos)
      continue;
    std::string k = token.substr(0, eq);
    std::string v = token.substr(eq + 1);
    if (k == "mmsi")       mmsi  = v;
    else if (k == "range") range = v;
    else if (k == "angle") angle = v;
  }

  if (mmsi.empty()) {
    std::cerr << "[TRAIL_FOLLOW] no mmsi in: " << value << std::endl;
    return;
  }

  std::string updates = "contact=AIS" + mmsi;
  if (!range.empty())
    updates += " # trail_range=" + range;
  if (!angle.empty())
    updates += " # trail_angle=" + angle;

  // A waypoint mission and a trail pursuit fight over the helm - BHV_Waypoint_mod
  // and BHV_AvdColregsV22 both run on WAYPOINT=true and outweigh the trail
  // behaviour - so engaging Follow ends the mission, the same way
  // AUTONOMOUS_STOP does.
  Notify("WAYPOINT", "false");
  Notify("STATIONING", "false");

  // Without this the helm computes a trail solution that never reaches the
  // actuators. AUTONOMOUS_START does the same thing for the same reason.
  Notify("MOOS_MANUAL_OVERRIDE", "false");

  Notify("TRAIL_UPDATES", updates);
  Notify("TRAILING", "true");
  std::cout << "[TRAIL_FOLLOW] " << updates << std::endl;
}

//---------------------------------------------------------
// Procedure: handleColavTest()
//   COLAV_TEST=true|false
//
//   Turning the test off also clears whatever targets are still out there -
//   leaving simulated AIS traffic running after the operator has closed the
//   test panel is how a test target ends up mistaken for real traffic.

void WebSocket::handleColavTest(const std::string& value)
{
  if (value == "true") {
    Notify("COLAV_TEST", "true");
  } else if (value == "false") {
    Notify("COLAV_TEST", "false");
    Notify("TRAFFIC_CLEAR", "all");
  }
}

//---------------------------------------------------------
// Procedure: handlePutTarget()
//   PUT_TARGET=lat=29.7231#lon=-91.1102#course=at_usv#speed=5.0#type=cargo
//
//   The UI thinks in lat/lon and knots; pTrafficSim thinks in the MOOS local
//   grid and m/s. This is the only place that conversion happens.
//
//   course=at_usv is passed through as hdg=at_ownship rather than resolved
//   here: pTrafficSim knows where the boat is at the instant it creates the
//   contact, and a heading computed one websocket round trip earlier would be
//   subtly stale.

void WebSocket::handlePutTarget(const std::string& value)
{
  if (!originReady("PUT_TARGET"))
    return;

  std::string lat_s, lon_s, course_s, speed_s, type_s, length_s, beam_s;

  std::stringstream ss(value);
  std::string token;
  while (std::getline(ss, token, '#')) {
    size_t eq = token.find('=');
    if (eq == std::string::npos)
      continue;
    std::string k = token.substr(0, eq);
    std::string v = token.substr(eq + 1);
    if (k == "lat")          lat_s = v;
    else if (k == "lon")     lon_s = v;
    else if (k == "course")  course_s = v;
    else if (k == "speed")   speed_s = v;
    else if (k == "type")    type_s = v;
    else if (k == "length")  length_s = v;
    else if (k == "beam")    beam_s = v;
  }

  if (lat_s.empty() || lon_s.empty()) {
    std::cerr << "[PUT_TARGET] no lat/lon in: " << value << std::endl;
    return;
  }

  double lat = 0, lon = 0, speed_kts = 5.0;
  try {
    lat = std::stod(lat_s);
    lon = std::stod(lon_s);
    if (!speed_s.empty())
      speed_kts = std::stod(speed_s);
  }
  catch (const std::exception& e) {
    std::cerr << "[PUT_TARGET] parse error in: " << value << std::endl;
    return;
  }

  if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
    std::cerr << "[PUT_TARGET] out-of-range coords: " << lat << "," << lon << std::endl;
    return;
  }

  // LatLong2LocalUTM returns north before east - the (y, x) argument order here
  // is correct, not a bug. USE_UTM is ON for this build, which is what makes
  // this the matching inverse of pTrafficSim's UTM2LatLong.
  double localY = 0.0, localX = 0.0;
  m_geodesy.LatLong2LocalUTM(lat, lon, localY, localX);

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2)
      << "x=" << localX << ",y=" << localY;
  oss << ",hdg=" << ((course_s == "at_usv" || course_s.empty()) ? "at_ownship" : course_s);
  oss << ",spd=" << (speed_kts / 1.94384);          // knots -> m/s
  oss << ",type=" << (type_s.empty() ? "cargo" : type_s);
  if (!length_s.empty()) oss << ",length=" << length_s;
  if (!beam_s.empty())   oss << ",beam=" << beam_s;

  Notify("TRAFFIC_SPAWN", oss.str());
  std::cout << "[PUT_TARGET] " << oss.str() << std::endl;
}

//---------------------------------------------------------
// Procedure: handleColavDist()
//   COLAV_DIST=pwt_outer_dist=300#pwt_inner_dist=150#completed_dist=1500
//              #max_util_cpa_dist=300#min_util_cpa_dist=100
//
//   The engagement distances of the collision avoidance behaviour, edited from
//   the Calibration tab. Only the five distances are accepted - this is not a
//   general back door into the behaviour's parameter set.

void WebSocket::handleColavDist(const std::string& value)
{
  double outer = colav_outer, inner = colav_inner, completed = colav_completed;
  double min_cpa = colav_min_cpa, max_cpa = colav_max_cpa;

  std::stringstream ss(value);
  std::string token;
  while (std::getline(ss, token, '#')) {
    size_t eq = token.find('=');
    if (eq == std::string::npos)
      continue;
    std::string k = token.substr(0, eq);
    double v = 0;
    try {
      v = std::stod(token.substr(eq + 1));
    }
    catch (const std::exception& e) {
      std::cerr << "[COLAV_DIST] parse error in: " << token << std::endl;
      return;
    }
    if (v < 0)
      continue;

    if (k == "pwt_outer_dist")           outer = v;
    else if (k == "pwt_inner_dist")      inner = v;
    else if (k == "completed_dist")      completed = v;
    else if (k == "min_util_cpa_dist")   min_cpa = v;
    else if (k == "max_util_cpa_dist")   max_cpa = v;
  }

  // The behaviour clamps these pairs itself and would silently drag the
  // partner value; do it here instead so what goes out is what comes back.
  if (inner > outer)         inner = outer;
  if (completed < outer)     completed = outer;
  if (min_cpa > max_cpa)     min_cpa = max_cpa;

  colav_outer     = outer;
  colav_inner     = inner;
  colav_completed = completed;
  colav_min_cpa   = min_cpa;
  colav_max_cpa   = max_cpa;

  applyColavDist();
}

//---------------------------------------------------------
// Procedure: applyColavDist()
//   Push the distances into the running avdcol_ instances and echo back what
//   is now in force.
//
//   Three postings, and all three are needed:
//
//   1. The behaviour's updates variable. No "name=" field, so it reaches every
//      spawned instance rather than one contact's. It must also stay clear of
//      the substring "name" altogether - BehaviorSpec::checkForSpawningStrings
//      tests for that substring, not for "name=", and would read the update as
//      a spawn request.
//
//   2. pContactMgrV20's alert range, which the behaviour set from its own
//      pwt_outer_dist/completed_dist at helm start and never revisits. Without
//      this a contact is not alerted until the old range, so the helm would
//      spawn the behaviour too late to avoid at the new distance.
//
//   3. COLAV_DIST, for the UI and the mission log.

void WebSocket::applyColavDist()
{
  std::ostringstream upd;
  upd << std::fixed << std::setprecision(1)
      << "pwt_outer_dist=" << colav_outer
      << "#pwt_inner_dist=" << colav_inner
      << "#completed_dist=" << colav_completed
      << "#max_util_cpa_dist=" << colav_max_cpa
      << "#min_util_cpa_dist=" << colav_min_cpa;
  Notify("CONTACT_INFO", upd.str());

  std::ostringstream alert;
  alert << std::fixed << std::setprecision(1)
        << "id=avdcol_, alert_range=" << colav_outer
        << ", cpa_range=" << colav_completed;
  Notify("BCM_ALERT_REQUEST", alert.str());

  std::ostringstream echo;
  echo << std::fixed << std::setprecision(1)
       << "pwt_outer_dist=" << colav_outer
       << ",pwt_inner_dist=" << colav_inner
       << ",completed_dist=" << colav_completed
       << ",min_util_cpa_dist=" << colav_min_cpa
       << ",max_util_cpa_dist=" << colav_max_cpa;
  colav_dist = echo.str();
  Notify("COLAV_DIST", colav_dist);

  std::cout << "[COLAV_DIST] " << colav_dist << std::endl;
}

//---------------------------------------------------------
// Procedure: handleClearTargets()
//   CLEAR_TARGETS=all | CLEAR_TARGETS=name=AIS366999901

void WebSocket::handleClearTargets(const std::string& value)
{
  Notify("TRAFFIC_CLEAR", value.empty() ? "all" : value);
}

// TRIM=1 one trim-up pulse, TRIM=-1 one trim-down pulse, TRIM=0 stop.
// iCAN turns each command into a short timed pulse (profile settings).
void WebSocket::handleTrim(const std::string& value)
{
  if (!isNumber(value)) {
    reportRunWarning("TRIM ignored, not a number: " + value);
    return;
  }
  Notify("TRIM", atof(value.c_str()));
}

void WebSocket::handleControleManual(const std::string& value)
{
  // MOOS_MANUAL_OVERRIDE (two Rs) is what pHelmIvP, planchaPID and iCAN
  // read. It used to be posted misspelled, so manual control never reached
  // them and planchaPID kept overwriting the operator's rudder.
  if (value == "true") {
    Notify("MOOS_MANUAL_OVERRIDE", "true");
  } else if (value == "false") {
    Notify("MOOS_MANUAL_OVERRIDE", "false");
    Notify("DESIRED_RUDDER", 0.0);
    Notify("DESIRED_THRUST", 0.0);
  }
}

void WebSocket::handleAutonomousControl(const std::string& value)
{
  Notify("MOOS_MANUAL_OVERIDE", "true");
}

void WebSocket::handleDesiredHeading(const std::string& value)
{
  double value_heading = std::stod(value);
  Notify("DESIRED_HEADING", value_heading);
}

void WebSocket::handleDesiredSpeed(const std::string& value)
{
  double value_speed = std::stod(value);
  Notify("WPT_UPDATE", "speed=" + std::to_string(value_speed));
}

void WebSocket::handleDesiredRudder(const std::string& value)
{
  if (!isNumber(value)) {
    reportRunWarning("DESIRED_RUDDER ignored, not a number: " + value);
    return;
  }
  Notify("DESIRED_RUDDER", atof(value.c_str()));
}

void WebSocket::handleDesiredThrust(const std::string& value)
{
  if (!isNumber(value)) {
    reportRunWarning("DESIRED_THRUST ignored, not a number: " + value);
    return;
  }
  Notify("DESIRED_THRUST", atof(value.c_str()));
}

void WebSocket::handleDesiredGear(const std::string& value)
{
  if (value == "1") {
    Notify("DESIRED_GEAR", "A");
  } else if (value == "0") {
    Notify("DESIRED_GEAR", "N");
  } else if (value == "-1") {
    Notify("DESIRED_GEAR", "R");
  } 
}

void WebSocket::handleAutonomousWaypoints(const std::string& value)
{
  if (!originReady("AUTONOMOUS_WAYPOINTS"))
    return;

  std::vector<std::string> coordTokens;
  
  std::istringstream tokenStream(value);
  std::string token;
  while (std::getline(tokenStream, token, ':')) {
      if (!token.empty()) {
          coordTokens.push_back(token);
      }
  }
  
  std::vector<std::string> convertedCoords;
  
  for (const auto &coordStr : coordTokens) {
      size_t commaPos = coordStr.find(',');
      if (commaPos == std::string::npos) {
          std::cerr << "Invalid coordinate format for: " << coordStr << std::endl;
          continue;
      }
      
      std::string latPart = coordStr.substr(0, commaPos);
      std::string lonPart = coordStr.substr(commaPos + 1);
      
      try {
          double orig_lon = std::stod(lonPart);
          double orig_lat = std::stod(latPart);
          
          double local_y = 0.0, local_x = 0.0;
          m_geodesy.LatLong2LocalUTM(orig_lat, orig_lon, local_y, local_x);
          
          std::ostringstream oss;
          oss << std::fixed << std::setprecision(1) << local_x << "," << local_y;
          convertedCoords.push_back(oss.str());
      } catch (const std::exception &e) {
          std::cerr << "Error converting coordinate (" << coordStr << "): " << e.what() << std::endl;
      }
  }
  
  std::ostringstream finalStream;
  finalStream << "xpoints="; //Need to be sent like this because of the behavior
  bool first = true;
  for (const auto &convertedCoord : convertedCoords) {
      if (!first) {
          finalStream << ":";
      }
      finalStream << convertedCoord;
      first = false;
  }
  
  std::string wpt_update_str = finalStream.str();
  Notify("WPT_UPDATE", wpt_update_str);
}

void WebSocket::handleAutonomousStart(const std::string& value)
{
  if (!originReady("AUTONOMOUS_START"))
    return;
  // New format: true#SPEED#WAYPOINTS#REPEAT=true/false
  // Example: true#10#42.3601,-71.0589:42.3602,-71.0590#REPEAT=true
  
  size_t firstHashPos = value.find('#');
  if (firstHashPos != std::string::npos) {
    std::string boolPart = value.substr(0, firstHashPos);
    std::string remainder = value.substr(firstHashPos + 1);
    
    size_t secondHashPos = remainder.find('#');
    
    if (boolPart == "true") {
      Notify("WAYPOINT", "true");
      Notify("MOOS_MANUAL_OVERRIDE", "false");

      // Parse speed
      if (secondHashPos != std::string::npos) {
        // Format: true#SPEED#WAYPOINTS or true#SPEED#WAYPOINTS#REPEAT=true/false
        std::string speedPart = remainder.substr(0, secondHashPos);
        std::string waypointsPart = remainder.substr(secondHashPos + 1);
        
        // Check if there's a REPEAT parameter
        size_t thirdHashPos = waypointsPart.find("#REPEAT=");
        std::string actualWaypointsPart = waypointsPart;
        bool hasRepeat = false;
        bool repeatValue = false;
        std::string idRoute = "";
        
        reportEvent("[AUTONOMOUS_START] thirdHashPos: " + std::to_string(thirdHashPos));
        
        if (thirdHashPos != std::string::npos) {
          // Extract waypoints part (before REPEAT)
          actualWaypointsPart = waypointsPart.substr(0, thirdHashPos);
          
          // Extract everything after REPEAT
          std::string afterRepeat = waypointsPart.substr(thirdHashPos + 8); // skip "#REPEAT="
          
          reportEvent("[AUTONOMOUS_START] afterRepeat: " + afterRepeat);
          
          // Check if there's an ID_ROUTE parameter
          size_t idRoutePos = afterRepeat.find("#ID_ROUTE=");
          std::string repeatPart = afterRepeat;
          
          if (idRoutePos != std::string::npos) {
            // Extract REPEAT value (before ID_ROUTE)
            repeatPart = afterRepeat.substr(0, idRoutePos);
            // Extract ID_ROUTE value (after #ID_ROUTE=)
            idRoute = afterRepeat.substr(idRoutePos + 10); // skip "#ID_ROUTE="
          }
          
          reportEvent("[AUTONOMOUS_START] repeatPart: '" + repeatPart + "'");
          
          if (repeatPart == "true") {
            repeatValue = true;
            hasRepeat = true;
            repeat_mission = true;
            Notify("REPEAT_MISSION", "true");
            reportEvent("[AUTONOMOUS_START] Set REPEAT_MISSION=true");
          } else if (repeatPart == "false") {
            repeatValue = false;
            hasRepeat = true;
            repeat_mission = false;
            Notify("REPEAT_MISSION", "false");
            reportEvent("[AUTONOMOUS_START] Set REPEAT_MISSION=false");
          }
        } else {
          // No REPEAT parameter, but check for ID_ROUTE directly
          size_t idRoutePos = waypointsPart.find("#ID_ROUTE=");
          if (idRoutePos != std::string::npos) {
            actualWaypointsPart = waypointsPart.substr(0, idRoutePos);
            idRoute = waypointsPart.substr(idRoutePos + 10); // skip "#ID_ROUTE="
          }
        }
        
        // Save ID_ROUTE if present
        if (!idRoute.empty()) {
          Notify("ID_ROUTE", idRoute);
          id_route = idRoute;
          std::cout << "[AUTONOMOUS_START] Set ID_ROUTE: " << idRoute << std::endl;
        }
        
        try {
          double value_speed = std::stod(speedPart);
          Notify("WPT_UPDATE", "speed=" + std::to_string(value_speed));
          std::cout << "[AUTONOMOUS_START] Set speed: " << value_speed << std::endl;
        } catch (const std::exception& e) {
          std::cerr << "Error parsing speed value: " << speedPart << std::endl;
        }
        
        // Parse and convert waypoints
        if (!actualWaypointsPart.empty()) {
          std::vector<std::string> coordTokens;
          std::istringstream tokenStream(actualWaypointsPart);
          std::string token;
          while (std::getline(tokenStream, token, ':')) {
            if (!token.empty()) {
              coordTokens.push_back(token);
            }
          }

          std::vector<std::string> convertedCoords;
          for (const auto &coordStr : coordTokens) {
            size_t commaPos = coordStr.find(',');
            if (commaPos == std::string::npos) {
              std::cerr << "Invalid coordinate format for: " << coordStr << std::endl;
              continue;
            }

            std::string latPart = coordStr.substr(0, commaPos);
            std::string lonPart = coordStr.substr(commaPos + 1);

            try {
              double orig_lon = std::stod(lonPart);
              double orig_lat = std::stod(latPart);

              double local_y = 0.0, local_x = 0.0;
              m_geodesy.LatLong2LocalUTM(orig_lat, orig_lon, local_y, local_x);

              std::ostringstream oss;
              oss << std::fixed << std::setprecision(1) << local_x << "," << local_y;
              convertedCoords.push_back(oss.str());
            } catch (const std::exception &e) {
              std::cerr << "Error converting coordinate (" << coordStr << "): " << e.what() << std::endl;
            }
          }

          std::ostringstream finalStream;
          finalStream << "polygon=";
          bool first = true;
          for (const auto &convertedCoord : convertedCoords) {
            if (!first) {
              finalStream << ":";
            }
            finalStream << convertedCoord;
            first = false;
          }

          std::string wpt_update_str = finalStream.str();
          Notify("WPT_UPDATE", wpt_update_str);
          std::cout << "[AUTONOMOUS_START] Set waypoints: " << coordTokens.size() << " points" << std::endl;
        }
        
        // Send repeat update LAST, after everything else, with a delay
        if (hasRepeat) {
          usleep(100000); // 100ms delay
          if (repeatValue) {
            Notify("WPT_UPDATE", "repeat=1000");
            Notify("REPEAT_MISSION", "true");
            std::cout << "[AUTONOMOUS_START] Set repeat: 1000 (infinite)" << std::endl;
          } else {
            Notify("WPT_UPDATE", "repeat=0");
            Notify("REPEAT_MISSION", "false");
            std::cout << "[AUTONOMOUS_START] Set repeat: 1 (once)" << std::endl;
          }
        }
      } else {
        // Old format: true#SPEED (no waypoints)
        try {
          double value_speed = std::stod(remainder);
          Notify("WPT_UPDATE", "speed=" + std::to_string(value_speed));
          std::cout << "[AUTONOMOUS_START] Set speed (legacy): " << value_speed << std::endl;
        } catch (const std::exception& e) {
          std::cerr << "Error parsing speed value: " << remainder << std::endl;
        }
      }
    }
  } else if (value == "true") {
    // Simple format without speed or waypoints
    Notify("WAYPOINT", "true");
    Notify("MOOS_MANUAL_OVERRIDE", "false");
    std::cout << "[AUTONOMOUS_START] Started (no speed/waypoints)" << std::endl;
  }
}

void WebSocket::handleAutonomousStop(const std::string& value)
{
  if (value == "true") {
    //Notify("MOOS_MANUAL_OVERRIDE", "true");
    Notify("WAYPOINT", "false");
    Notify("STATIONING", "false"); //To end the station keep and not keep the mission stuck

    //Stop the engine and put rudder in zero just for safety
    Notify("DESIRED_RUDDER", 0.0);
    Notify("DESIRED_THRUST", 0.0);
  }
}

void WebSocket::handleAutonomousReturnToBase(const std::string& value)
{
  if (value == "true") {
    Notify("RETURN", "true");
  } else if (value == "false") {
    Notify("RETURN", "false");
  }
}

void WebSocket::handleReturnUpdate(const std::string& value)
{
  if (!originReady("RETURN_UPDATE"))
    return;

  auto eqPos = value.find('=');
  if (eqPos == std::string::npos) {
      MOOSTrace("RETURN_UPDATE malformed: missing '=' in \"%s\"", value.c_str());
      return;
  }
  std::string payload = value.substr(eqPos + 1);

  auto commaPos = payload.find(',');
  if (commaPos == std::string::npos) {
      MOOSTrace("RETURN_UPDATE malformed: missing ',' in \"%s\"", payload.c_str());
      return;
  }
  std::string latStr = payload.substr(0, commaPos);
  std::string lonStr = payload.substr(commaPos + 1);

  double latDeg = 0.0, lonDeg = 0.0;
  try {
      latDeg = std::stod(latStr);
      lonDeg = std::stod(lonStr);
  }
  catch (const std::invalid_argument& e) {
      MOOSTrace("RETURN_UPDATE parse error: invalid number \"%s\" or \"%s\"", latStr.c_str(), lonStr.c_str());
      return;
  }
  catch (const std::out_of_range& e) {
      MOOSTrace("RETURN_UPDATE parse error: value out of range \"%s\" or \"%s\"", latStr.c_str(), lonStr.c_str());
      return;
  }

  if (latDeg < -90.0 || latDeg > 90.0 || lonDeg < -180.0 || lonDeg > 180.0) {
      MOOSTrace("RETURN_UPDATE out‑of‑range coords: lat=%f lon=%f", latDeg, lonDeg);
      return;
  }

  double localY = 0.0, localX = 0.0;
  m_geodesy.LatLong2LocalUTM(latDeg, lonDeg, localY, localX);

  std::ostringstream oss;
  oss << "points="
      << std::fixed << std::setprecision(1)
      << localX << "," << localY;
  Notify("RETURN_UPDATE", oss.str());
}

void WebSocket::handleStopCan(const std::string& value)
{
  if (value == "true") {
    Notify("STOP_CAN", "true");
  } else if (value == "false") {
    Notify("STOP_CAN", "false");
  }
}

void WebSocket::handleConstantHeading(const std::string& value)
{
  if (value == "true") {
    Notify("CONSTANT_HEADING", "true");
  } else if (value == "false") {
    Notify("CONSTANT_HEADING", "false");
  }
}

void WebSocket::handleSetpointHeading(const std::string& value)
{
  value_setpoint = std::stod(value);
  Notify("SETPOINT_HEADING", value_setpoint);
}

void WebSocket::handleHeadingKp(const std::string& value)
{
  double value_heading_kp = std::stod(value);
  Notify("HEADING_KP", value_heading_kp);
}

void WebSocket::handleHeadingKi(const std::string& value)
{
  double value_heading_ki = std::stod(value);
  Notify("HEADING_KI", value_heading_ki);
}

void WebSocket::handleHeadingKd(const std::string& value)
{
  double value_heading_kd = std::stod(value);
  Notify("HEADING_KD", value_heading_kd);
}

void WebSocket::handleSpeedKp(const std::string& value)
{
  double value_speed_kp = std::stod(value);
  Notify("SPEED_KP", value_speed_kp);
}

void WebSocket::handleSpeedKi(const std::string& value)
{
  double value_speed_ki = std::stod(value);
  Notify("SPEED_KI", value_speed_ki);
}

void WebSocket::handleSpeedKd(const std::string& value)
{
  double value_speed_kd = std::stod(value);
  Notify("SPEED_KD", value_speed_kd);
}

// Service mode: while active planchaPID applies the HEADING_K*/SPEED_K* gains
// pushed from the UI instead of its built-in gain schedule.
void WebSocket::handleServiceMode(const std::string& value)
{
  if (value == "true") {
    Notify("SERVICE_MODE", "true");
  } else if (value == "false") {
    Notify("SERVICE_MODE", "false");
  }
}

void WebSocket::handleStationKeepWaypoint(const std::string& value)
{
  if (!originReady("STATION_KEEP_WAYPOINT"))
    return;
  // Parse the value to check if it's TRUE or FALSE
  size_t hashPos = value.find('#');

  if (hashPos != std::string::npos) {
    // Format: TRUE#INNER_RADIUS=50#OUTER_RADIUS=100#COORDS=29.726252,-91.165742
    std::string boolPart = value.substr(0, hashPos);

    if (boolPart == "TRUE") {
      // Extract parameters
      std::string params = value.substr(hashPos + 1);

      double inner_radius = 50.0;  // default
      double outer_radius = 100.0; // default
      double local_x = 0.0;
      double local_y = 0.0;

      // Parse INNER_RADIUS
      size_t innerPos = params.find("INNER_RADIUS=");
      if (innerPos != std::string::npos) {
        size_t startPos = innerPos + 13; // length of "INNER_RADIUS="
        size_t endPos = params.find('#', startPos);
        std::string innerStr = (endPos != std::string::npos) ?
                                params.substr(startPos, endPos - startPos) :
                                params.substr(startPos);
        try {
          inner_radius = std::stod(innerStr);
        } catch (const std::exception& e) {
          std::cerr << "Error parsing inner_radius: " << innerStr << std::endl;
        }
      }

      // Parse OUTER_RADIUS
      size_t outerPos = params.find("OUTER_RADIUS=");
      if (outerPos != std::string::npos) {
        size_t startPos = outerPos + 13; // length of "OUTER_RADIUS="
        size_t endPos = params.find('#', startPos);
        std::string outerStr = (endPos != std::string::npos) ?
                                params.substr(startPos, endPos - startPos) :
                                params.substr(startPos);
        try {
          outer_radius = std::stod(outerStr);
        } catch (const std::exception& e) {
          std::cerr << "Error parsing outer_radius: " << outerStr << std::endl;
        }
      }

      // Parse COORDS and convert to local
      size_t coordsPos = params.find("COORDS=");
      if (coordsPos != std::string::npos) {
        size_t startPos = coordsPos + 7; // length of "COORDS="
        size_t endPos = params.find('#', startPos);
        std::string coordsStr = (endPos != std::string::npos) ?
                                 params.substr(startPos, endPos - startPos) :
                                 params.substr(startPos);

        size_t commaPos = coordsStr.find(',');
        if (commaPos != std::string::npos) {
          std::string latStr = coordsStr.substr(0, commaPos);
          std::string lonStr = coordsStr.substr(commaPos + 1);

          try {
            double orig_lat = std::stod(latStr);
            double orig_lon = std::stod(lonStr);

            m_geodesy.LatLong2LocalUTM(orig_lat, orig_lon, local_y, local_x);
          } catch (const std::exception& e) {
            std::cerr << "Error parsing coordinates: " << coordsStr << std::endl;
          }
        }
      }

      //Activate the StationKeep behavior
      Notify("STATIONING", "true");

      // Send STATION_UPDATES
      Notify("STATION_UPDATES", "inner_radius=" + std::to_string(inner_radius));
      Notify("STATION_UPDATES", "outer_radius=" + std::to_string(outer_radius));

      std::ostringstream oss;
      oss << "station_pt=" << std::fixed << std::setprecision(1) << local_x << "," << local_y;
      Notify("STATION_UPDATES", oss.str());

      Notify("STATION_UPDATES", "pwt=250");

      // Send WPT_UPDATE
      Notify("WPT_UPDATE", "pwt=50");
    }
  } else if (value == "FALSE") {
    //Deactivate the StaionKeep behavior
    Notify("STATIONING", "false");
    // Send updates for FALSE case
    Notify("STATION_UPDATES", "pwt=50");
    Notify("WPT_UPDATE", "pwt=250");
  }
}

void WebSocket::handleHeartbeatPing(const std::string& value)
{
  // The connection timestamp is already updated when any message is received
  // via the UpdateConnectionTime() call in the message handler
  // This handler just acknowledges the heartbeat ping
  reportEvent("Received heartbeat ping from client: " + value);
}

void WebSocket::handleRepeat(const std::string& value)
{
  // Convert the repeat count from string to integer to validate
  try {
    int repeat_count = std::stoi(value);
    
    // Send the repeat parameter to MOOS-IvP via WPT_UPDATE
    Notify("WPT_UPDATE", "repeat=" + value);
    
    std::cout << "[REPEAT] Set repeat count: " << repeat_count << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error parsing repeat value: " << value << " - " << e.what() << std::endl;
  }
}

void WebSocket::handleBaseCoords(const std::string& value)
{
  // Save the base coordinates value
  base_coords = value;
  
  // Notify MOOS with the BASE_COORDS value
  Notify("BASE_COORDS", value);
  
  std::cout << "[BASE_COORDS] Saved and notified: " << value << std::endl;
}

void WebSocket::handleCanSend(const std::string& value)
{
  // Parse the value to check if it's true or false
  if (value == "true") {
    can_send = true;
    Notify("CAN_SEND", "true");
    std::cout << "[CAN_SEND] Set to true" << std::endl;
  } else if (value == "false") {
    can_send = false;
    Notify("CAN_SEND", "false");
    std::cout << "[CAN_SEND] Set to false" << std::endl;
  }
}

void WebSocket::handleRegionUpdates(const std::string& value)
{
  // Expected value format: core_poly=pts={lat,lon:lat,lon:lat,lon:...}
  // Parse the pts={...} block, convert lat/lon to local x,y,
  // and rebuild the string with local coordinates.

  reportEvent("[REGION_UPDATES] Raw input: [" + value + "]");
  reportEvent("[REGION_UPDATES] Input length: " + std::to_string(value.length()));

  if (!originReady("REGION_UPDATES"))
    return;

  // Find the pts={...} block
  size_t ptsStart = value.find("pts={");
  if (ptsStart == std::string::npos) {
    reportRunWarning("[REGION_UPDATES] Could not find pts={} in value: " + value);
    return;
  }

  // Everything before "pts={"
  std::string prefix = value.substr(0, ptsStart);

  size_t braceOpen = value.find('{', ptsStart);
  size_t braceClose = value.find('}', braceOpen);
  if (braceOpen == std::string::npos || braceClose == std::string::npos) {
    reportRunWarning("[REGION_UPDATES] Malformed pts block in value: " + value);
    return;
  }

  // Extract the coordinate string inside braces
  std::string coordsStr = value.substr(braceOpen + 1, braceClose - braceOpen - 1);

  // Everything after the closing brace (could be empty)
  std::string suffix = value.substr(braceClose + 1);

  // Split coordinates by ':'
  std::vector<std::string> coordTokens;
  std::istringstream tokenStream(coordsStr);
  std::string token;
  while (std::getline(tokenStream, token, ':')) {
    if (!token.empty()) {
      coordTokens.push_back(token);
    }
  }

  // Convert each lat,lon pair to local x,y
  std::vector<std::string> convertedCoords;
  for (const auto &coordStr : coordTokens) {
    size_t commaPos = coordStr.find(',');
    reportEvent("[REGION_UPDATES] Processing coord token: [" + coordStr + "]");
    if (commaPos == std::string::npos) {
      std::cerr << "[REGION_UPDATES] Invalid coordinate format: " << coordStr << std::endl;
      continue;
    }

    std::string latPart = coordStr.substr(0, commaPos);
    std::string lonPart = coordStr.substr(commaPos + 1);

    try {
      double orig_lat = std::stod(latPart);
      double orig_lon = std::stod(lonPart);

      double local_y = 0.0, local_x = 0.0;
      m_geodesy.LatLong2LocalUTM(orig_lat, orig_lon, local_y, local_x);

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1) << local_x << "," << local_y;
      convertedCoords.push_back(oss.str());
    } catch (const std::exception &e) {
      std::cerr << "[REGION_UPDATES] Error converting coordinate (" << coordStr << "): " << e.what() << std::endl;
    }
  }

  // Remove closing vertex if polygon is a closed ring
  // (last point == first point), as XYPolygon expects open polygons
  if (convertedCoords.size() > 1 && 
      convertedCoords.front() == convertedCoords.back()) {
    convertedCoords.pop_back();
  }

  // Validate: need at least 3 points for a convex polygon
  if (convertedCoords.size() < 3) {
    reportRunWarning("[REGION_UPDATES] Only " + std::to_string(convertedCoords.size()) 
                     + " unique coords, need at least 3");
    return;
  }

  // Rebuild the pts={...} string with local coordinates
  std::ostringstream finalStream;
  finalStream << prefix << "pts={";
  bool first = true;
  for (const auto &convertedCoord : convertedCoords) {
    if (!first) {
      finalStream << ":";
    }
    finalStream << convertedCoord;
    first = false;
  }
  finalStream << "}" << suffix;

  std::string region_update_str = finalStream.str();
  Notify("REGION_UPDATES", region_update_str);
  reportEvent("[REGION_UPDATES] Published: " + region_update_str);
}

void WebSocket::handleRegion(const std::string& value)
{
  // Expected format: true#REGION_UPDATES=core_poly=pts={lat,lon:lat,lon:...}
  //              or: false#REGION_UPDATES=core_poly=pts={lat,lon:lat,lon:...}

  // Split on the first '#'
  size_t hashPos = value.find('#');

  std::string regionBool;
  std::string remainder;

  if (hashPos != std::string::npos) {
    regionBool = value.substr(0, hashPos);
    remainder = value.substr(hashPos + 1);
  } else {
    regionBool = value;
  }

  // Set the REGION variable
  if (regionBool == "true") {
    Notify("REGION", "true");
  } else if (regionBool == "false") {
    Notify("REGION", "false");
  }

  // Process REGION_UPDATES if present
  if (!remainder.empty()) {
    // remainder should be: REGION_UPDATES=core_poly=pts={...}
    size_t eqPos = remainder.find('=');
    if (eqPos != std::string::npos) {
      std::string key = remainder.substr(0, eqPos);
      std::string regionValue = remainder.substr(eqPos + 1);

      if (key == "REGION_UPDATES" && !regionValue.empty()) {
        handleRegionUpdates(regionValue);
      }
    }
  }
}

//---------------------------------------------------------
// Procedure: originReady()
//   Commands that carry lat/lon need the mission origin. Without it they
//   are refused (with a warning) rather than converted with a guess.

bool WebSocket::originReady(const std::string& what)
{
  if (m_origin_ok)
    return true;
  reportRunWarning(what + " ignored: LatOrigin / LongOrigin missing from the mission file");
  return false;
}
