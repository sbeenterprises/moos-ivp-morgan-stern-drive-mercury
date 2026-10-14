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
#include "AngleUtils.h"
#include "CPAEngine.h"
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
#include <fstream>
#include <cmath>
#include <cstdio>

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

// The helm names each spawned avoidance instance after the template in
// alpha.bhv plus the contact: avdcol_AIS366999901.
const std::string colav_bhv_prefix = "avdcol_";

// How long a new collision alert waits for the behaviour to classify the
// encounter (see updateColavAlerts()) before it is raised anyway, MOOS s.
const double colav_classify_wait = 2.0;

// The behaviour reports none:none until it has classified an encounter, and
// complete:* once it is over; neither says what the boat is doing.
static bool colavModeKnown(const std::string& mode)
{
  return((mode != "") && !strBegins(mode, "none") && !strBegins(mode, "complete"));
}

// Limit AIS contacts: the range around the boat inside which the collision
// avoidance engages a contact. Anything beyond it is held off through
// pContactMgrV20's disable variable, which reaches the spawned avdcol_
// instance as BHV_ABLE_FILTER and parks it (IvPBehavior returns the "disabled"
// activity state, so it builds no objective function). Reversible: a contact
// that comes back inside the ring is re-enabled.
//
// This does not stop the contact manager alerting, so a behaviour can still be
// spawned for a held contact after the fact - hence the periodic re-post.
double       ais_radius = 0;           // metres; 0 = off, every contact engaged
std::string  ais_limit_report;         // echoed to the UI
std::string  ais_disable_var = "XYZ_DISABLE_TARGET";
std::string  ais_enable_var  = "XYZ_REENABLE_TARGET";
double       ais_last_apply  = 0;
double       ais_last_repost = 0;

// The rest of each report is kept for the collision alerts (COLAV_ALERTS).
// spd is in NODE_REPORT units, m/s - iDivisorNMEA converts the AIS knots.
struct AisContact {
  double x, y, utc;
  double lat, lon;
  double hdg, spd, len;
  std::string type;
};
std::map<std::string, AisContact> ais_contacts;  // every contact heard from
std::set<std::string> ais_held;                  // the ones currently held off

double nav_x = 0, nav_y = 0;
bool   nav_xy_known = false;

// NODE_REPORT contact variables
std::string contact_name = "";
double contact_lat = 0;
double contact_lon = 0;
double contact_spd = 0;
double contact_hdg = 0;

//---------------------------------------------------------
// Procedure: parseLatLon()
//   "29.722820,-91.215390" - the format of BASE_COORDS, from the UI and in
//   base_file. False for anything that is not two numbers in range.

static bool parseLatLon(const std::string& str, double& lat, double& lon)
{
  size_t comma = str.find(',');
  if(comma == std::string::npos)
    return(false);
  std::string slat = stripBlankEnds(str.substr(0, comma));
  std::string slon = stripBlankEnds(str.substr(comma + 1));
  if(!isNumber(slat) || !isNumber(slon))
    return(false);
  lat = atof(slat.c_str());
  lon = atof(slon.c_str());
  return((lat >= -90) && (lat <= 90) && (lon >= -180) && (lon <= 180));
}


//---------------------------------------------------------
// Constructor()

WebSocket::WebSocket()
{
  msg_recebida = "";
  m_connection_timeout = 60.0; // Default 60 seconds timeout
  m_last_connection_check = 0.0;
  m_heartbeat_interval = 30.0; // Send heartbeat every 30 seconds (less than frontend's 60s timeout)
  m_last_heartbeat_time = 0.0;
  m_server = nullptr;
  m_port = 8000;

  m_comms_loss_return        = true;
  m_comms_loss_timeout       = 30.0;
  m_comms_loss_speed         = 5.0;
  m_comms_loss_always        = false;
  m_underway_speed           = 1.0;
  m_base_lat   = 29.72282;
  m_base_lon   = -91.21539;
  m_lat_origin = 29.71970895316288;
  m_lon_origin = -91.14705165281887;

  m_last_ui_rx.store(0.0);
  m_last_ui_rx_real.store(0.0);
  m_ui_seen.store(false);
  m_comms_lost          = false;
  m_comms_loss_handled  = false;
  m_comms_lost_at       = 0;
  m_last_loss_duration  = 0;
  m_deferred_manual_at  = 0;
  m_comms_loss_count    = 0;
  m_return_active       = false;
  m_hold_active         = false;
  m_return_by_failsafe  = false;
  m_return_idle_since   = 0;
  m_base_x = 0;
  m_base_y = 0;
  m_base_announced = false;

  m_colav_alerts_json  = "[]";
  m_colav_alerts_built = 0;
  m_colav_alerts_dirty = true;
  m_colav_alert_hold   = 15.0;
  m_nav_speed_to_mps   = 0.514444;   // knots, the boat's GPS

  initializeMessageHandlers();
}

// Implement the new methods for multiple clients.
//
// Add/Remove/UpdateConnectionTime run on the websocket threads, one per
// client, while Iterate() reads the same containers on the MOOS thread. So
// every access goes through m_clients_mutex, and the appcast events these
// produce are queued for Iterate() rather than reported from here - the
// appcast is not thread-safe either.
void WebSocket::AddWebSocket(ix::WebSocket* ws)
{
  if (ws != nullptr) {
    std::lock_guard<std::mutex> lock(m_clients_mutex);
    m_webSockets.push_back(ws);
    m_lastMessageTime[ws] = MOOSTime(); // Initialize with current time

    // Initialize heartbeat time when first client connects
    if (m_webSockets.size() == 1)
      m_last_heartbeat_time = MOOSTime();

    m_client_events.push_back("New WebSocket client connected. Total clients: " + std::to_string(m_webSockets.size()));
  }
  // An open connection is a frame from the UI as much as a message is
  NoteUiFrame();
}

void WebSocket::RemoveWebSocket(ix::WebSocket* ws)
{
  std::lock_guard<std::mutex> lock(m_clients_mutex);
  m_lastMessageTime.erase(ws); // Remove timestamp tracking
  auto it = std::find(m_webSockets.begin(), m_webSockets.end(), ws);
  if (it != m_webSockets.end()) {
    m_webSockets.erase(it);
    m_client_events.push_back("WebSocket client disconnected. Total clients: " + std::to_string(m_webSockets.size()));
  }
}

bool WebSocket::HasConnectedClients() const
{
  std::lock_guard<std::mutex> lock(m_clients_mutex);
  return !m_webSockets.empty();
}

size_t WebSocket::GetConnectedClientCount() const
{
  std::lock_guard<std::mutex> lock(m_clients_mutex);
  return m_webSockets.size();
}

void WebSocket::UpdateConnectionTime(ix::WebSocket* ws)
{
  if (ws != nullptr) {
    std::lock_guard<std::mutex> lock(m_clients_mutex);
    m_lastMessageTime[ws] = MOOSTime();

    // A client dropped as stale is still connected - it had only gone quiet.
    // Once it is heard from again it goes back on the broadcast list, rather
    // than staying connected without telemetry until the UI's own stale check
    // gives up on the socket and reconnects.
    if (std::find(m_webSockets.begin(), m_webSockets.end(), ws) == m_webSockets.end()) {
      m_webSockets.push_back(ws);
      m_client_events.push_back("Stale WebSocket client heard from again. Total clients: " + std::to_string(m_webSockets.size()));
    }
  }
  NoteUiFrame();
}

void WebSocket::NoteUiFrame()
{
  m_last_ui_rx.store(MOOSTime());
  m_last_ui_rx_real.store(MOOSLocalTime(false));
  m_ui_seen.store(true);
}

void WebSocket::CheckStaleConnections()
{
  double current_time = MOOSTime();
  std::vector<std::string> warnings;
  {
    std::lock_guard<std::mutex> lock(m_clients_mutex);
    for (auto it = m_webSockets.begin(); it != m_webSockets.end(); ) {
      double time_since_last_msg = current_time - m_lastMessageTime[*it];
      if (time_since_last_msg > m_connection_timeout) {
        warnings.push_back("Detected stale connection (" +
                           std::to_string(time_since_last_msg) +
                           "s since last message). Timeout: " +
                           std::to_string(m_connection_timeout) + "s");
        // Off the broadcast list, but the timestamp stays: a client that is
        // heard from again is put back by UpdateConnectionTime().
        it = m_webSockets.erase(it);
      }
      else
        ++it;
    }
  }

  for (const std::string& w : warnings)
    reportRunWarning(w);
}

void WebSocket::SetServer(ix::WebSocketServer* server, int port)
{
  m_server = server;
  m_port = port;
}

void WebSocket::SendMessage(const std::string& message, bool binary)
{
  std::set<ix::WebSocket*> live;
  {
    std::lock_guard<std::mutex> lock(m_clients_mutex);
    live.insert(m_webSockets.begin(), m_webSockets.end());
  }

  if (live.empty() || (m_server == nullptr))
  {
    reportRunWarning("Attempted to send message but no WebSocket clients connected");
    return;
  }

  // Broadcast to the connected clients that are not stale. The server's list
  // holds shared pointers, which keep each connection alive while it is
  // written to. A failed send can run the close callback on this thread
  // (RemoveWebSocket), which is why no lock is held here.
  for (const auto& client : m_server->getClients())
  {
    if (live.count(client.get()) > 0)
      client->send(message, binary);
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
    else if(key == "NAV_X") {
      nav_x = msg.GetDouble();
      nav_xy_known = true;
    }
    else if(key == "NAV_Y")
      nav_y = msg.GetDouble();
    else if(key == "DESIRED_HEADING")
      desired_heading = msg.GetDouble();
    else if(key == "DESIRED_SPEED")
      desired_speed = msg.GetDouble();
    else if(key == "DESIRED_RUDDER")
      desired_rudder = msg.GetDouble();
    else if(key == "DESIRED_THRUST")
      desired_thrust = msg.GetDouble();
    else if(key == "IVPHELM_STATE") {
      if(msg.GetString() != ivphelm_state)
        m_colav_alerts_dirty = true;
      ivphelm_state = msg.GetString();
    }
    else if(key == "IVPHELM_BHV_ACTIVE") {
      // Posted by the helm on every change: the behaviours that produced an
      // objective function on its last iteration, comma separated.
      m_helm_avoiding.clear();
      vector<string> bhvs = parseString(msg.GetString(), ',');
      for(unsigned int i=0; i<bhvs.size(); i++) {
        string bhv = stripBlankEnds(bhvs[i]);
        if((bhv.size() > colav_bhv_prefix.size()) && strBegins(bhv, colav_bhv_prefix))
          m_helm_avoiding.insert(bhv.substr(colav_bhv_prefix.size()));
      }
      m_colav_alerts_dirty = true;
    }
    else if(key == "COLAV_BHV_TRACE") {
      // avdcol_'s runxflag, every helm iteration while an instance runs:
      // the COLREGS mode it is in for that contact. A new mode for a contact
      // being avoided goes out on the next tick rather than the next second -
      // going in extremis is the one the operator most needs to see.
      string cn   = tokStringParse(msg.GetString(), "cn", ',', '=');
      string mode = tokStringParse(msg.GetString(), "mode", ',', '=');
      if((cn != "") && (mode != "")) {
        if(m_helm_avoiding.count(cn) && (m_colav_mode[cn] != mode))
          m_colav_alerts_dirty = true;
        m_colav_mode[cn] = mode;
      }
    }
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
    else if(key == "BASE_COORDS") {
      // Also settable from the MOOSDB (uPokeDB, another app), not only from
      // the UI. Our own posts come back here too and are skipped: a late echo
      // of the previous base would otherwise undo one just set from the UI.
      double lat = 0, lon = 0;
      if((msg.GetSource() != GetAppName()) && (msg.GetString() != base_coords) &&
         parseLatLon(msg.GetString(), lat, lon))
        setBase(lat, lon, "moosdb");
    }
    else if(key == "RETURN")
      m_return_active = (tolower(msg.GetString()) == "true");
    else if(key == "RETURN_HOLD")
      m_hold_active = (tolower(msg.GetString()) == "true");
    else if(key == "STATIONING")
      m_stationing = tolower(msg.GetString());
    else if(key == "NAV_SPEED")
      nav_speed = msg.GetDouble();
    else if(key == "CAN_SEND")
      can_send = (msg.GetString() == "true");
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
    else if(key == "AIS_RADIUS") {
      // Also settable straight from the MOOSDB (uPokeDB, a script, another
      // app), not only from the panel. Our own echo comes back here too,
      // which the comparison makes a no-op.
      double v = msg.IsDouble() ? msg.GetDouble()
                                : atof(msg.GetString().c_str());
      if(v != ais_radius) {
        ais_radius = v;
        applyAisLimit(true);
      }
    }
    else if(key == "NODE_REPORT") {
      // Parse NODE_REPORT to extract NAME, X, Y, SPD, HDG
      std::string node_report = msg.GetString();

      std::string name = "";
      std::string type = "";
      double x = 0, y = 0;
      double spd = 0, hdg = 0, len = 0;

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
          } else if (key_part == "LENGTH") {
            len = atof(val_part.c_str());
          } else if (key_part == "TYPE") {
            type = val_part;
          }
        }
      }

      // Convert local X,Y to lat/lon, using the same projection as
      // pMarineViewer and pNodeReporter (see USE_UTM in the top-level
      // CMakeLists). This used the flat-earth grid plus a hand-tuned
      // "y += 100" that cancelled most - not all - of the resulting error;
      // the projection is the actual fix, so the fudge is gone.
      double lat_origin = 29.71970895316288;
      double lon_origin = -91.14705165281887;

      CMOOSGeodesy geodesy;
      geodesy.Initialise(lat_origin, lon_origin);

      double lat, lon;
#ifdef USE_UTM
      geodesy.UTM2LatLong(x, y, lat, lon);
#else
      geodesy.LocalGrid2LatLong(x, y, lat, lon);
#endif

      // Keep every contact, not just the last one: the AIS limit ranks them
      // against each other, so it needs the whole picture.
      if(name != "") {
        AisContact cn;
        cn.x = x;
        cn.y = y;
        cn.utc = MOOSTime();
        cn.lat = lat;
        cn.lon = lon;
        cn.hdg = hdg;
        cn.spd = spd;
        cn.len = len;
        cn.type = type;
        ais_contacts[name] = cn;
      }

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
   // Post the base again on the next tick, also after a reconnection
   m_base_announced = false;
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

  // Events queued by the websocket threads (connects, disconnects)
  std::vector<std::string> client_events;
  {
    std::lock_guard<std::mutex> lock(m_clients_mutex);
    client_events.swap(m_client_events);
  }
  for (const std::string& event : client_events)
    reportEvent(event);

  // Tell the helm where the base is. Once at startup - the MOOSDB hands the
  // value to pHelmIvP whenever it registers, so the order the apps come up in
  // does not matter - and again on every change of base.
  if (!m_base_announced)
    m_base_announced = postBaseToHelm();

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

  applyAisLimit(false);
  updateColavAlerts();

  // After the inbox: a command that arrived with the link coming back has
  // already been dispatched by the time the loss is judged.
  checkCommsLoss();

  // Manual commands held back after ending a return (see postManual)
  if(!m_deferred_manual.empty() && (current_time >= m_deferred_manual_at)) {
    for(const auto& kv : m_deferred_manual)
      Notify(kv.first, kv.second);
    m_deferred_manual.clear();
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
  root["AIS_RADIUS"] = ais_limit_report;
  root["COMMS_LOSS"] = commsLossJson();
  root["COLAV_ALERTS"] = m_colav_alerts_json;

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
    // Must match pContactMgrV20's disable_var / enable_var. That is where the
    // AIS cap does its work - the contact manager turns a post here into
    // BHV_ABLE_FILTER, which parks the avdcol_ instance for that contact.
    else if(param == "ais_disable_var" && value != "") {
      ais_disable_var = value;
      handled = true;
    }
    else if(param == "ais_enable_var" && value != "") {
      ais_enable_var = value;
      handled = true;
    }
    // Read by main() before the server starts listening; only accepted here
    // so it is not flagged as an unknown parameter.
    else if(param == "port" && isNumber(value))
      handled = true;
    // Comms-loss failsafe. See checkCommsLoss().
    else if(param == "comms_loss_return")
      handled = setBooleanOnString(m_comms_loss_return, value);
    else if(param == "comms_loss_timeout")
      handled = setPosDoubleOnString(m_comms_loss_timeout, value);
    else if(param == "comms_loss_speed")
      handled = setPosDoubleOnString(m_comms_loss_speed, value);
    else if(param == "comms_loss_always")
      handled = setBooleanOnString(m_comms_loss_always, value);
    else if(param == "comms_loss_underway_speed")
      handled = setNonNegDoubleOnString(m_underway_speed, value);
    else if(param == "base_lat" && isNumber(value)) {
      m_base_lat = atof(value.c_str());
      handled = true;
    }
    else if(param == "base_lon" && isNumber(value)) {
      m_base_lon = atof(value.c_str());
      handled = true;
    }
    else if(param == "base_file" && value != "") {
      m_base_file = value;
      handled = true;
    }
    // Collision alerts. NAV_SPEED is knots from the GPS on the boat and m/s
    // in uSimMarineV22; the CPA needs it in m/s, the units of the contacts.
    else if(param == "nav_speed_units") {
      string units = tolower(value);
      if((units == "knots") || (units == "kts") || (units == "kn")) {
        m_nav_speed_to_mps = 0.514444;
        handled = true;
      }
      else if((units == "mps") || (units == "m/s")) {
        m_nav_speed_to_mps = 1.0;
        handled = true;
      }
    }
    else if(param == "colav_alert_hold")
      handled = setNonNegDoubleOnString(m_colav_alert_hold, value);
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

  // The UI sends a heartbeat every 5 s and the server pings every 5 s, so
  // anything much shorter than this would declare a loss on a healthy link.
  if(m_comms_loss_timeout < 10) {
    reportConfigWarning("comms_loss_timeout below 10 s, using 10 s");
    m_comms_loss_timeout = 10;
  }

  // The base is converted to the helm's local grid around the mission origin,
  // the same one pHelmIvP and pNodeReporter use.
  double origin = 0;
  if(m_MissionReader.GetValue("LatOrigin", origin))
    m_lat_origin = origin;
  if(m_MissionReader.GetValue("LongOrigin", origin))
    m_lon_origin = origin;

  // The configured base, unless one set from the UI was saved to base_file
  if(!setBase(m_base_lat, m_base_lon, "config"))
    reportConfigWarning("base_lat/base_lon rejected, no base until one is set from the UI");
  if(m_base_file != "")
    loadBaseFile();

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
  Register("NAV_X", 0); //Ownship local grid position, for ranging contacts
  Register("NAV_Y", 0);
  Register("AIS_RADIUS", 0); //Range inside which the avoidance engages contacts
  Register("RETURN", 0);      //Return to base behaviour running
  Register("RETURN_HOLD", 0); //...arrived, holding at the base
  Register("STATIONING", 0);  //Station keep active, for the underway test
  Register("NAV_SPEED", 0);   //...and the boat's speed
  Register("IVPHELM_BHV_ACTIVE", 0); //Behaviours steering the boat - avdcol_ ones are collision alerts
  Register("COLAV_BHV_TRACE", 0);    //COLREGS mode of each running avdcol_ instance

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
  m_msgs << "  Porta:    " << m_port << endl;
  m_msgs << "  Estado:   Running" << endl;
  m_msgs << endl;

  // Exemplo de uso do ACTable para formatar alguns parâmetros
  ACTable actab(2);
  actab << "Parâmetro | Valor";
  actab.addHeaderLines();
  actab << "Address" << "0.0.0.0";
  actab << "Port"    << std::to_string(m_port);
  actab << "Status"  << "Running";
  actab << "Connected Clients" << std::to_string(GetConnectedClientCount());
  actab << "Mensagem Recebida"  << msg_recebida;
  actab << "Connection Timeout" << std::to_string(m_connection_timeout) + "s";
  
  // Show connection health
  {
    std::lock_guard<std::mutex> lock(m_clients_mutex);
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
  }

  m_msgs << actab.getFormattedString();

  // Comms-loss failsafe
  double now = MOOSTime();
  std::string link = "waiting for the first UI";
  if (m_ui_seen.load()) {
    link = m_comms_lost ? "LOST" : "ok";
    link += ", last frame " + doubleToString(now - m_last_ui_rx.load(), 1) + " s ago";
  }
  std::string ret = m_return_active ? "returning" : (m_hold_active ? "holding at base" : "idle");
  if (m_return_reason != "")
    ret += " (" + m_return_reason + ")";

  m_msgs << endl << endl << "Comms-loss failsafe" << endl;
  ACTable fstab(2);
  fstab << "Item | Value";
  fstab.addHeaderLines();
  fstab << "Return on loss" << std::string(m_comms_loss_return ? "on" : "off") +
            (m_comms_loss_always ? ", always" : ", helm in DRIVE or underway");
  fstab << "Timeout" << doubleToString(m_comms_loss_timeout, 1) + " s";
  fstab << "Return speed" << doubleToString(m_comms_loss_speed, 2);
  fstab << "UI link" << link;
  fstab << "Losses" << std::to_string(m_comms_loss_count) +
            ((m_comms_loss_count > 0) ? ", last action: " + m_comms_loss_action : "");
  fstab << "Helm, underway" << (ivphelm_state == "" ? std::string("?") : ivphelm_state) +
            (isUnderway() ? ", underway" : ", not underway");
  std::string stay = stayReason();
  fstab << "On a loss now" << (stay == "" ? std::string("return to base") : "stay (" + stay + ")");
  fstab << "Return" << ret;
  fstab << "Base" << (base_coords == "" ? std::string("none") :
            base_coords + " (" + m_base_source + ")");
  fstab << "Base, local" << "x=" + doubleToString(m_base_x, 1) + ", y=" + doubleToString(m_base_y, 1);
  if (m_base_file != "")
    fstab << "Base file" << m_base_file;
  m_msgs << fstab.getFormattedString();

  // Collision alerts, as sent to the UI
  m_msgs << endl << endl << "Collision alerts (COLAV_ALERTS): "
         << m_colav_alerts.size() << ", hold " << doubleToString(m_colav_alert_hold, 0)
         << " s, NAV_SPEED x" << doubleToString(m_nav_speed_to_mps, 3) << " = m/s" << endl;
  if (!m_colav_alerts.empty()) {
    ACTable catab(4);
    catab << "Contact | State | Mode | For";
    catab.addHeaderLines();
    for (const auto& kv : m_colav_alerts) {
      catab << kv.first << (kv.second.active ? "avoiding" : "ended")
            << ((kv.second.mode == "") ? std::string("-") : kv.second.mode)
            << doubleToString(now - kv.second.since, 0) + " s";
    }
    m_msgs << catab.getFormattedString();
  }

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
  m_messageHandlers["AIS_RADIUS"] = &WebSocket::handleAisRadius;
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
  cancelReturn("TRAIL_FOLLOW");

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
  static constexpr double kLatOrigin = 29.71970895316288;
  static constexpr double kLonOrigin = -91.14705165281887;
  m_geodesy.Initialise(kLatOrigin, kLonOrigin);

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
// Procedure: handleAisRadius()
//   AIS_RADIUS=500  - engage contacts within 500 m, hold the rest off
//   AIS_RADIUS=0    - off, every contact is engaged
//
//   A range is what makes a single manoeuvre watchable: with a dozen contacts
//   in the picture the helm is solving all of them at once and no one
//   encounter can be read off the chart. The UI draws the same ring around the
//   boat, so what is in play is visible rather than inferred.

void WebSocket::handleAisRadius(const std::string& value)
{
  double radius = 0;
  try {
    radius = std::stod(value);
  }
  catch (const std::exception& e) {
    std::cerr << "[AIS_RADIUS] not a number: " << value << std::endl;
    return;
  }
  if (radius < 0)
    radius = 0;
  if (radius > 20000)
    radius = 20000;

  ais_radius = radius;
  Notify("AIS_RADIUS", ais_radius);
  std::cout << "[AIS_RADIUS] " << ais_radius << std::endl;

  // Apply at once rather than waiting for the next tick, and force the posts
  // even where nothing changed - lifting the limit has to release every
  // contact it was holding.
  applyAisLimit(true);
}

//---------------------------------------------------------
// Procedure: applyAisLimit()
//   Hold off every contact beyond the ring, release the ones inside it.
//
//   force: re-post the whole picture even where nothing changed. Used when the
//   radius itself moves; otherwise only transitions are posted.
//
//   Runs at 1 Hz. Ranging on every tick would be wasted work - contacts move
//   metres per second - and the re-post below is deliberately periodic anyway:
//   a contact held off now may not have spawned its behaviour yet, and the one
//   that spawns later comes up enabled.

void WebSocket::applyAisLimit(bool force)
{
  double now = MOOSTime();
  if (!force && ((now - ais_last_apply) < 1.0))
    return;
  ais_last_apply = now;

  // Every few seconds the whole held set is posted again, not just the
  // contacts that changed side: a contact held off before its behaviour was
  // spawned would otherwise come up enabled and quietly rejoin the picture.
  bool repost = (now - ais_last_repost) >= 3.0;
  if (repost)
    ais_last_repost = now;

  // Contacts that have stopped reporting are no longer anyone's problem.
  // Released on the way out: if the same name comes back it starts even.
  for (auto it = ais_contacts.begin(); it != ais_contacts.end(); ) {
    if ((now - it->second.utc) > 60.0) {
      if (ais_held.erase(it->first) > 0)
        Notify(ais_enable_var, it->first);
      it = ais_contacts.erase(it);
    }
    else
      ++it;
  }

  std::ostringstream rep;
  rep << std::fixed << std::setprecision(0) << "radius=" << ais_radius
      << ",tracked=" << ais_contacts.size();

  if (ais_radius <= 0) {
    for (const std::string& cn : ais_held)
      Notify(ais_enable_var, cn);
    ais_held.clear();
    rep << ",engaged=" << ais_contacts.size() << ",held=0";
    ais_limit_report = rep.str();
    return;
  }

  // Without an ownship position there is nothing to range against, so leave
  // the picture alone rather than hold contacts off at random.
  if (!nav_xy_known) {
    rep << ",engaged=0,held=0,why=no_ownship_position";
    ais_limit_report = rep.str();
    return;
  }

  unsigned int engaged = 0;
  double nearest_held = -1;
  for (const auto& kv : ais_contacts) {
    const std::string& cn = kv.first;
    double range = std::hypot(kv.second.x - nav_x, kv.second.y - nav_y);

    if (range <= ais_radius) {
      engaged++;
      if (ais_held.erase(cn) > 0)
        Notify(ais_enable_var, cn);
    }
    else {
      if (force || repost || (ais_held.count(cn) == 0))
        Notify(ais_disable_var, cn);
      ais_held.insert(cn);
      if ((nearest_held < 0) || (range < nearest_held))
        nearest_held = range;
    }
  }

  rep << ",engaged=" << engaged << ",held=" << ais_held.size();
  // How far out the nearest held contact is, so the panel can say what the
  // radius would have to reach to take the next one in.
  if (nearest_held >= 0)
    rep << ",next=" << nearest_held;
  ais_limit_report = rep.str();
}

//---------------------------------------------------------
// Procedure: handleClearTargets()
//   CLEAR_TARGETS=all | CLEAR_TARGETS=name=AIS366999901

void WebSocket::handleClearTargets(const std::string& value)
{
  Notify("TRAFFIC_CLEAR", value.empty() ? "all" : value);
}

void WebSocket::handleTrim(const std::string& value)
{
  double value_trim = std::stod(value);
  Notify("TRIM", value_trim);
}

void WebSocket::handleControleManual(const std::string& value)
{
  if (value == "true") {
    cancelReturn("CONTROLE_MANUAL");
    Notify("MOOS_MANUAL_OVERIDE", "true");
  } else if (value == "false") {
    Notify("MOOS_MANUAL_OVERIDE", "false");
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

// Manual remote control. Moving the rudder or thrust from the UI is the
// operator taking over, so it ends a return to base - planchaPID would
// otherwise keep overwriting these with the return's own commands.
void WebSocket::handleDesiredRudder(const std::string& value)
{
  double value_rudder = std::stod(value);
  postManual("DESIRED_RUDDER", value_rudder, cancelReturn("manual control (DESIRED_RUDDER)"));
}

void WebSocket::handleDesiredThrust(const std::string& value)
{
  double value_thrust = std::stod(value);
  postManual("DESIRED_THRUST", value_thrust, cancelReturn("manual control (DESIRED_THRUST)"));
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
  double lat_origin = 29.71970895316288;
  double lon_origin = -91.14705165281887;
  
  m_geodesy.Initialise(lat_origin, lon_origin);

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
  // New format: true#SPEED#WAYPOINTS#REPEAT=true/false
  // Example: true#10#42.3601,-71.0589:42.3602,-71.0590#REPEAT=true
  
  // A mission start, with or without waypoints, ends a return to base
  if (value.compare(0, 4, "true") == 0)
    cancelReturn("AUTONOMOUS_START");

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
          double lat_origin = 29.71970895316288;
          double lon_origin = -91.14705165281887;
          m_geodesy.Initialise(lat_origin, lon_origin);

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
    cancelReturn("AUTONOMOUS_STOP");
    //Notify("MOOS_MANUAL_OVERRIDE", "true");
    Notify("WAYPOINT", "false");
    Notify("STATIONING", "false"); //To end the station keep and not keep the mission stuck

    //Stop the engine and put rudder in zero just for safety
    Notify("DESIRED_RUDDER", 0.0);
    Notify("DESIRED_THRUST", 0.0);
  }
}

// The same return the comms-loss watchdog engages, on demand: to the base
// iWebSocket holds, at comms_loss_speed, then holding there.
void WebSocket::handleAutonomousReturnToBase(const std::string& value)
{
  if (value == "true") {
    if (base_coords == "")
      reportRunWarning("AUTONOMOUS_RETURN_TO_BASE ignored, no base set");
    else
      engageReturn("operator");
  } else if (value == "false") {
    cancelReturn("AUTONOMOUS_RETURN_TO_BASE=false");
  }
}

void WebSocket::handleReturnUpdate(const std::string& value)
{
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

  // The return point is the base: going through setBase() keeps the hold
  // point, BASE_COORDS and base_file in step with it.
  setBase(latDeg, lonDeg, "ui");
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
  // Parse the value to check if it's TRUE or FALSE
  size_t hashPos = value.find('#');

  if (hashPos != std::string::npos) {
    // Format: TRUE#INNER_RADIUS=50#OUTER_RADIUS=100#COORDS=29.726252,-91.165742
    std::string boolPart = value.substr(0, hashPos);

    if (boolPart == "TRUE") {
      cancelReturn("STATION_KEEP_WAYPOINT");

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
        double lat_origin = 29.71970895316288;
        double lon_origin = -91.14705165281887;

        m_geodesy.Initialise(lat_origin, lon_origin);

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

//---------------------------------------------------------
// Procedure: handleBaseCoords()
//   BASE_COORDS=29.722820,-91.215390  (lat,lon)
//
//   The base set on the chart. It is where the boat goes when the link to
//   the UI is lost, so it is converted and handed to the helm at once, not
//   only stored for the UI to read back.

void WebSocket::handleBaseCoords(const std::string& value)
{
  double lat = 0, lon = 0;
  if(!parseLatLon(value, lat, lon)) {
    reportRunWarning("BASE_COORDS rejected, expected lat,lon: " + value);
    return;
  }
  setBase(lat, lon, "ui");
  std::cout << "[BASE_COORDS] " << base_coords << std::endl;
}

//---------------------------------------------------------
// Procedure: setBase()
//   Validate a new base, convert it to the helm's local grid and schedule
//   the posts to the helm. A base from the UI or the MOOSDB is also saved to
//   base_file, so a restart at sea does not quietly go back to the default.

bool WebSocket::setBase(double lat, double lon, const std::string& source)
{
  if((lat < -90) || (lat > 90) || (lon < -180) || (lon > 180) ||
     ((lat == 0) && (lon == 0))) {
    reportRunWarning("Base rejected (" + source + "): " +
                     doubleToString(lat, 6) + "," + doubleToString(lon, 6));
    return(false);
  }

  // Same projection as the NODE_REPORT conversion above (USE_UTM)
  CMOOSGeodesy geodesy;
  geodesy.Initialise(m_lat_origin, m_lon_origin);
  double north = 0, east = 0;
#ifdef USE_UTM
  geodesy.LatLong2LocalUTM(lat, lon, north, east);
#else
  geodesy.LatLong2LocalGrid(lat, lon, north, east);
#endif

  // A base hundreds of km from the mission origin is a typo or swapped
  // coordinates, not somewhere to send an unmanned boat.
  if((fabs(north) > 200000) || (fabs(east) > 200000)) {
    reportRunWarning("Base rejected (" + source + "), more than 200 km from the mission origin: " +
                     doubleToString(lat, 6) + "," + doubleToString(lon, 6));
    return(false);
  }

  m_base_lat = lat;
  m_base_lon = lon;
  m_base_x = east;
  m_base_y = north;
  m_base_source = source;

  std::ostringstream coords;
  coords << std::fixed << std::setprecision(6) << lat << "," << lon;
  base_coords = coords.str();

  if((source == "ui") || (source == "moosdb"))
    saveBaseFile();

  // Posted on the next tick: from OnStartUp() the MOOSDB may not be
  // connected yet, and after that the next tick is 50 ms away.
  m_base_announced = false;

  reportEvent("Base (" + source + "): " + base_coords + " = x=" +
              doubleToString(m_base_x, 1) + ", y=" + doubleToString(m_base_y, 1));
  return(true);
}

//---------------------------------------------------------
// Procedure: postBaseToHelm()
//   The return behaviour (comms_return in alpha.bhv) goes to the base at the
//   failsafe speed; the hold behaviour (comms_hold) keeps station on it.

bool WebSocket::postBaseToHelm()
{
  if(base_coords == "")
    return(true);   // no valid base yet, nothing to post

  std::ostringstream ret;
  ret << std::fixed << std::setprecision(1)
      << "point=" << m_base_x << "," << m_base_y
      << " # speed=" << std::setprecision(2) << m_comms_loss_speed;

  std::ostringstream hold;
  hold << std::fixed << std::setprecision(1)
       << "station_pt=" << m_base_x << "," << m_base_y;

  // The base on pMarineViewer's chart, where the boat goes if the link to the
  // UI is lost. Same label every time, so a new base replaces the old marker.
  std::ostringstream marker;
  marker << std::fixed << std::setprecision(1)
         << "type=diamond,x=" << m_base_x << ",y=" << m_base_y
         << ",width=15,color=red,label=comms_loss_base,msg=BASE (comms loss)";

  bool ok = Notify("RETURN_UPDATE", ret.str());
  ok = Notify("RETURN_HOLD_UPDATE", hold.str()) && ok;
  ok = Notify("BASE_COORDS", base_coords) && ok;
  ok = Notify("VIEW_MARKER", marker.str()) && ok;
  return(ok);
}

//---------------------------------------------------------
// Procedure: loadBaseFile() / saveBaseFile()
//   base_file holds a single line, lat,lon. Written to a temporary file and
//   renamed over the old one, so a power cut mid-write cannot leave half a
//   base behind.

bool WebSocket::loadBaseFile()
{
  std::ifstream in(m_base_file.c_str());
  if(!in.is_open())
    return(false);   // nothing saved yet: the configured base stands

  std::string line;
  std::getline(in, line);
  double lat = 0, lon = 0;
  if(!parseLatLon(line, lat, lon)) {
    reportConfigWarning("base_file " + m_base_file + " ignored, expected lat,lon: " + line);
    return(false);
  }
  return(setBase(lat, lon, "file"));
}

void WebSocket::saveBaseFile() const
{
  if(m_base_file == "")
    return;

  std::string tmp = m_base_file + ".tmp";
  {
    std::ofstream out(tmp.c_str(), std::ios::trunc);
    if(!out.is_open()) {
      std::cerr << "[BASE_COORDS] cannot write " << tmp << std::endl;
      return;
    }
    out << base_coords << std::endl;
  }
  if(std::rename(tmp.c_str(), m_base_file.c_str()) != 0)
    std::cerr << "[BASE_COORDS] cannot replace " << m_base_file << std::endl;
}

//---------------------------------------------------------
// Procedure: isUnderway()
//   Whether the boat is doing anything a lost link should interrupt: a mode
//   running on the helm, thrust commanded, or moving (adrift included).

bool WebSocket::isUnderway() const
{
  if((waypoint == "true") || (m_stationing == "true") || trailing)
    return(true);
  if(fabs(desired_thrust) > 0)
    return(true);
  return(nav_speed > m_underway_speed);
}

//---------------------------------------------------------
// Procedure: stayReason()
//   Why the boat would stay where it is if the link were lost now, or empty
//   if it would return to base. It returns whenever the helm is in DRIVE -
//   NothingToDo included - or the boat is underway. With the helm in PARK
//   and the boat idle (the boat at the dock after power-up, start_in_drive =
//   false) it stays, unless comms_loss_always is set.

std::string WebSocket::stayReason() const
{
  if(!m_comms_loss_return)
    return("comms_loss_return is off");
  if(base_coords == "")
    return("no base");
  if(m_comms_loss_always || (ivphelm_state == "DRIVE") || isUnderway())
    return("");
  return("helm in " + (ivphelm_state == "" ? std::string("no state") : ivphelm_state) + ", not underway");
}

//---------------------------------------------------------
// Procedure: checkCommsLoss()
//   The comms-loss watchdog, once per tick. Armed by the first frame from a
//   UI; the link counts as lost once nothing at all - no message, ping or
//   pong - has come from any UI for comms_loss_timeout seconds.
//
//   On a loss the boat returns to the base - see stayReason() for when it
//   does not. A boat left where it is is still watched: if it starts to drift,
//   or the helm goes to DRIVE, the return is engaged then. When the link comes
//   back the return carries on - it ends with the operator's next command, not
//   by itself.

void WebSocket::checkCommsLoss()
{
  double now = MOOSTime();

  // Who engaged the current return stops mattering once it is over. Not on
  // the first idle tick: the arrival endflags post RETURN=false and
  // RETURN_HOLD=true together, and the mail can split them.
  if(m_return_active || m_hold_active)
    m_return_idle_since = 0;
  else if(m_return_idle_since == 0)
    m_return_idle_since = now;
  else if((now - m_return_idle_since) > 3) {
    m_return_by_failsafe = false;
    m_return_reason = "";
  }

  if(!m_ui_seen.load())
    return;

  double last_rx = m_last_ui_rx.load();
  double silent  = now - last_rx;

  if(!m_comms_lost) {
    if(silent <= m_comms_loss_timeout)
      return;
    m_comms_lost = true;
    m_comms_loss_handled = false;
    m_comms_lost_at = m_last_ui_rx_real.load();
    m_comms_loss_count++;
    m_comms_loss_action = "";
    Notify("COMMS_LOSS", "true");
    reportRunWarning("Comms with the UI lost");
    reportEvent("Comms with the UI lost: nothing heard for " + doubleToString(silent, 1) + " s");
  }
  else if(silent <= m_comms_loss_timeout) {
    m_comms_lost = false;
    m_last_loss_duration = MOOSLocalTime(false) - m_comms_lost_at;
    Notify("COMMS_LOSS", "false");
    retractRunWarning("Comms with the UI lost");
    reportEvent("Comms with the UI restored after " + doubleToString(m_last_loss_duration, 1) + " s");
    return;
  }

  if(m_comms_loss_handled)
    return;

  std::string action;
  std::string stay = stayReason();
  if(m_return_active || m_hold_active) {
    action = "none, already returning to base";
    m_comms_loss_handled = true;
  }
  else if(stay == "") {
    engageReturn("comms_loss");
    action = "return to base";
    m_comms_loss_handled = true;
  }
  else if(!m_comms_loss_return || (base_coords == "")) {
    action = "none, " + stay;
    m_comms_loss_handled = true;
  }
  else
    action = "waiting, " + stay;   // looked at again every tick

  if(action != m_comms_loss_action) {
    m_comms_loss_action = action;
    reportEvent("Comms loss: " + action);
  }
}

//---------------------------------------------------------
// Procedure: engageReturn()
//   Hand the helm to the return behaviour. Everything that would fight it is
//   stood down first - for the helm (the mission, station keep, trail) and
//   for planchaPID (constant heading would steer a fixed heading instead of
//   the helm's).

void WebSocket::engageReturn(const std::string& reason)
{
  bool failsafe = (reason == "comms_loss");

  postBaseToHelm();   // the target before the start, never a stale base

  Notify("WAYPOINT", "false");
  Notify("STATIONING", "false");
  Notify("TRAILING", "false");
  Notify("CONSTANT_HEADING", "false");
  // Nobody is watching a failsafe return: run it on the configured gains,
  // not on ones left over from a calibration session.
  if(failsafe)
    Notify("SERVICE_MODE", "false");
  Notify("RETURN_HOLD", "false");
  Notify("RETURN", "true");
  Notify("MOOS_MANUAL_OVERRIDE", "false");   // the helm drives (PARK -> DRIVE)

  m_deferred_manual.clear();   // a manual command still held back is overtaken
  m_return_active = true;
  m_hold_active = false;
  m_return_by_failsafe = failsafe;
  m_return_reason = reason;
  m_return_idle_since = 0;

  reportEvent("Returning to base " + base_coords + " (" + reason + ")");
  std::cout << "[RETURN] to " << base_coords << " (" << reason << ")" << std::endl;
}

//---------------------------------------------------------
// Procedure: cancelReturn()
//   An operator command ends the return, or the hold at the base. Thrust and
//   rudder go to zero first, as the behaviours' own endflags do: planchaPID
//   stops publishing once RETURN/RETURN_HOLD drop, and its last command
//   would otherwise stay on the actuators. The command that called this
//   posts its own values straight after.

bool WebSocket::cancelReturn(const std::string& reason)
{
  if(!m_return_active && !m_hold_active)
    return(false);

  Notify("RETURN", "false");
  Notify("RETURN_HOLD", "false");
  Notify("DESIRED_THRUST", 0.0);
  Notify("DESIRED_RUDDER", 0.0);

  m_return_active = false;
  m_hold_active = false;
  m_return_by_failsafe = false;
  m_return_reason = "";

  reportEvent("Return to base ended by " + reason);
  std::cout << "[RETURN] ended by " << reason << std::endl;
  return(true);
}

//---------------------------------------------------------
// Procedure: postManual()
//   A manual rudder/thrust command from the UI. If it has just ended a
//   return, it is held back for 0.3 s: planchaPID posts zero thrust and
//   rudder when it lets go of the actuators, and that zero must not be the
//   last word over the operator's command. Commands arriving meanwhile
//   replace the held ones and go out together.

void WebSocket::postManual(const std::string& var, double value, bool cancelled)
{
  if(cancelled || !m_deferred_manual.empty()) {
    if(m_deferred_manual.empty())
      m_deferred_manual_at = MOOSTime() + 0.3;
    m_deferred_manual[var] = value;
    return;
  }
  Notify(var, value);
}

//---------------------------------------------------------
// Procedure: commsLossJson()
//   The failsafe as the UI sees it. LOST is only ever true for a moment on
//   the UI side - while it is true nothing reaches the UI - so what matters
//   there is RETURN/FAILSAFE and the record of the last loss.

Json::Value WebSocket::commsLossJson() const
{
  Json::Value cl;
  cl["ENABLED"]  = m_comms_loss_return;
  cl["TIMEOUT"]  = m_comms_loss_timeout;
  cl["ARMED"]    = m_ui_seen.load();
  // What a loss now would do, so the UI can tell the operator beforehand
  std::string stay = stayReason();
  cl["WILL_RETURN"] = (stay == "");
  cl["STAY_REASON"] = stay;
  cl["LOST"]     = m_comms_lost;
  cl["COUNT"]    = m_comms_loss_count;
  cl["LOST_AT"]  = m_comms_lost_at;          // last frame before the loss, UNIX s
  cl["DURATION"] = m_last_loss_duration;     // of the last loss that ended, s (wall clock)
  cl["ACTION"]   = m_comms_loss_action;
  cl["RETURN"]   = m_return_active ? "returning" : (m_hold_active ? "holding" : "idle");
  cl["FAILSAFE"] = m_return_by_failsafe;
  cl["REASON"]   = m_return_reason;
  return(cl);
}

//---------------------------------------------------------
// Procedure: updateColavAlerts()
//   The contacts the helm is steering around, for the Alert List in the UI.
//
//   Taken from the helm itself: an avdcol_ instance is in IVPHELM_BHV_ACTIVE
//   when it produced an objective function on the helm's last iteration, so
//   the avoidance is part of the heading and speed the boat is being given.
//   A contact that is only being watched - inside the alert range but outside
//   the behaviour's envelope, or held off by the AIS limit - is not an alert.
//   The helm stops posting the list when it leaves DRIVE, so the last one is
//   only believed while IVPHELM_STATE is DRIVE.
//
//   An alert stays up for colav_alert_hold seconds after the avoidance ends,
//   marked inactive. The behaviour drops in and out of ACTIVE as the geometry
//   moves through its envelope, and without the hold every flicker would
//   raise the alert on the operator's screen again.
//
//   The COLREGS mode comes from the behaviour's trace, which runs one helm
//   iteration behind IVPHELM_BHV_ACTIVE: a contact that has only just become
//   active still reads none:none. An alert keeps the last mode the behaviour
//   classified its avoidance as, and a new one is held back until there is
//   one - otherwise the operator reads "classifying" for a second, and an
//   encounter that is in extremis from the start sounds a warning before its
//   alarm. One the behaviour never classifies goes out after
//   colav_classify_wait all the same.
//
//   Range, bearing, CPA and TCPA are worked out here from the contact's last
//   NODE_REPORT and the boat's NAV_*, with the CPAEngine the behaviour uses -
//   on reported positions, where the behaviour extrapolates. -1 is unknown.
//   TCPA is 0 once the range is opening; the CPA is then the current range.
//
//   Rebuilt when the active set or the mode of a contact being avoided
//   changes, and once a second otherwise.

void WebSocket::updateColavAlerts()
{
  double now = MOOSTime();
  if(!m_colav_alerts_dirty && ((now - m_colav_alerts_built) < 1.0))
    return;
  m_colav_alerts_dirty = false;
  m_colav_alerts_built = now;

  std::set<std::string> avoiding;
  if(ivphelm_state == "DRIVE")
    avoiding = m_helm_avoiding;

  // A contact back to active inside its hold carries on the same alert
  for(const std::string& cn : avoiding) {
    ColavAlert& alert = m_colav_alerts[cn];
    if(alert.since == 0)
      alert.since = now;
    alert.active = true;
    alert.last_active = now;
  }
  for(auto it = m_colav_alerts.begin(); it != m_colav_alerts.end(); ) {
    if(avoiding.count(it->first) == 0) {
      if(it->second.active) {
        it->second.active = false;
        it->second.last_active = now;
      }
      if((now - it->second.last_active) >= m_colav_alert_hold) {
        it = m_colav_alerts.erase(it);
        continue;
      }
    }
    ++it;
  }
  for(auto it = m_colav_mode.begin(); it != m_colav_mode.end(); ) {
    if((m_colav_alerts.count(it->first) == 0) && (ais_contacts.count(it->first) == 0))
      it = m_colav_mode.erase(it);
    else
      ++it;
  }

  auto r1 = [](double v) { return(std::round(v * 10) / 10); };

  Json::Value list(Json::arrayValue);
  for(auto& kv : m_colav_alerts) {
    const std::string& cn = kv.first;
    ColavAlert& alert = kv.second;

    auto mode = m_colav_mode.find(cn);
    if(alert.active && (mode != m_colav_mode.end()) && colavModeKnown(mode->second))
      alert.mode = mode->second;
    if(alert.active && (alert.mode == "") && ((now - alert.since) < colav_classify_wait))
      continue;

    Json::Value a;
    a["cn"] = cn;
    // AIS contacts are named AIS<mmsi> by iDivisorNMEA and pTrafficSim alike;
    // the UI looks the ship's name up by the MMSI.
    Json::UInt mmsi = 0;
    if(strBegins(cn, "AIS") && isNumber(cn.substr(3), false))
      mmsi = (Json::UInt) atol(cn.substr(3).c_str());
    a["mmsi"] = mmsi;
    a["active"] = alert.active;
    a["for"] = (Json::Int) std::round(now - alert.since);
    a["mode"] = (alert.mode == "") ? std::string("none") : alert.mode;

    double range = -1, bng = -1, rel_bng = -1, cpa = -1, tcpa = -1;
    auto rep = ais_contacts.find(cn);
    if(rep != ais_contacts.end()) {
      const AisContact& c = rep->second;
      a["hdg"]  = r1(angle360(c.hdg));
      a["spd"]  = std::round(c.spd * 100) / 100;
      a["lat"]  = c.lat;
      a["lon"]  = c.lon;
      a["len"]  = r1(c.len);
      a["type"] = c.type;
      a["age"]  = r1(now - c.utc);
      if(nav_xy_known) {
        range   = std::hypot(c.x - nav_x, c.y - nav_y);
        bng     = relAng(nav_x, nav_y, c.x, c.y);
        rel_bng = angle360(bng - nav_heading);
        double osv = std::max(0.0, nav_speed * m_nav_speed_to_mps);
        CPAEngine engine(c.y, c.x, c.hdg, c.spd, nav_y, nav_x);
        cpa  = engine.evalCPA(nav_heading, osv, 3600);
        tcpa = engine.evalTimeCPA(nav_heading, osv, 3600);
      }
    }
    else {
      a["hdg"] = -1;
      a["spd"] = -1;
      a["age"] = -1;
    }
    a["range"]  = r1(range);
    a["bng"]    = r1(bng);
    a["relBng"] = r1(rel_bng);
    a["cpa"]    = r1(cpa);
    a["tcpa"]   = r1(tcpa);
    list.append(a);
  }

  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  writer["precision"] = 6;
  writer["precisionType"] = "decimal";
  std::string json = Json::writeString(writer, list);
  if(json != m_colav_alerts_json) {
    m_colav_alerts_json = json;
    Notify("COLAV_ALERTS", json);
  }
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

  double lat_origin = 29.71970895316288;
  double lon_origin = -91.14705165281887;
  m_geodesy.Initialise(lat_origin, lon_origin);

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
