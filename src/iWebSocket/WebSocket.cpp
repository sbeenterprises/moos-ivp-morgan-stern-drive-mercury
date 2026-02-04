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
double value_setpoint = 0;
double wpt_index = 0; // Index of the current waypoint
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
std::string wpt_stat;
std::string view_point;
std::string id_route;
bool repeat_mission = false;
std::string base_coords;


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
  initializeMessageHandlers();
}

// Implement the new methods for multiple clients
void WebSocket::AddWebSocket(ix::WebSocket* ws)
{
  if (ws != nullptr) {
    m_webSockets.push_back(ws);
    m_lastMessageTime[ws] = MOOSTime(); // Initialize with current time
    
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
    else if(key == "IVPHELM_STATE")
      ivphelm_state = msg.GetString();
    else if(key == "WPT_INDEX")
      wpt_index = msg.GetDouble();
    else if(key == "IVPHELM_ALLSTOP")
      ivphelm_allstop = msg.GetString();
    else if(key == "WPT_STAT")
      wpt_stat = msg.GetString();
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

      // Convert local X,Y to lat/lon
      double lat_origin = 29.71970895316288;
      double lon_origin = -91.14705165281887;

      CMOOSGeodesy geodesy;
      geodesy.Initialise(lat_origin, lon_origin);

      // OFfset to y by 100meters
      y +=100;

      double lat, lon;
      geodesy.LocalGrid2LatLong(x, y, lat, lon);

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
  
  // Aqui você pode incluir outras operações se necessário

  //Analysis the variable coming from Freeboard
  //Only process if there's a new message
  if (!msg_recebida.empty()) {
    //Split in the equal sign
    size_t equalsPos = msg_recebida.find('=');

    //take the two sides
    std::string key = msg_recebida.substr(0, equalsPos);
    std::string value = msg_recebida.substr(equalsPos + 1);

    std::cout << "Key: " << key << "\n";
    std::cout << "Value: " << value << "\n";

    // State machine approach: O(log n) lookup instead of O(n) linear search
    auto handler = m_messageHandlers.find(key);
    if (handler != m_messageHandlers.end()) {
      (this->*(handler->second))(value);
    }
    
    // Clear the message after processing to prevent re-execution
    msg_recebida = "";
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
  root["WPT_STAT"] = wpt_stat;
  root["VIEW_POINT"] = view_point;
  root["ID_ROUTE"] = id_route;
  root["REPEAT_MISSION"] = repeat_mission;
  root["DESIRED_SPEED"] = desired_speed;
  root["BASE_COORDS"] = base_coords;
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
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

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
  Register("BASE_COORDS", 0);

  
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
  m_messageHandlers["TRIM"] = &WebSocket::handleTrim;
  m_messageHandlers["STATION_KEEP_WAYPOINT"] = &WebSocket::handleStationKeepWaypoint;
  m_messageHandlers["HEARTBEAT_PING"] = &WebSocket::handleHeartbeatPing;
  m_messageHandlers["REPEAT"] = &WebSocket::handleRepeat;
  m_messageHandlers["BASE_COORDS"] = &WebSocket::handleBaseCoords;
}

void WebSocket::handleTrim(const std::string& value)
{
  double value_trim = std::stod(value);
  Notify("TRIM", value_trim);
}

void WebSocket::handleControleManual(const std::string& value)
{
  if (value == "true") {
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

void WebSocket::handleDesiredRudder(const std::string& value)
{
  double value_rudder = std::stod(value);
  Notify("DESIRED_RUDDER", value_rudder);
}

void WebSocket::handleDesiredThrust(const std::string& value)
{
  double value_thrust = std::stod(value);
  Notify("DESIRED_THRUST", value_thrust);
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
}

void WebSocket::handleAutonomousStart(const std::string& value)
{
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
    Notify("MOOS_MANUAL_OVERRIDE", "true");

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
  static constexpr double kLatOrigin = 29.71970895316288;
  static constexpr double kLonOrigin = -91.14705165281887;
  m_geodesy.Initialise(kLatOrigin, kLonOrigin);

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

void WebSocket::handleStationKeepWaypoint(const std::string& value)
{
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

void WebSocket::handleBaseCoords(const std::string& value)
{
  // Save the base coordinates value
  base_coords = value;
  
  // Notify MOOS with the BASE_COORDS value
  Notify("BASE_COORDS", value);
  
  std::cout << "[BASE_COORDS] Saved and notified: " << value << std::endl;
}

