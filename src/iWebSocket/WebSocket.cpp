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
double value_setpoint = 0;
double wpt_index = 0; // Index of the current waypoint
std::string ivphelm_state;
std::string ivphelm_allstop;
std::string wpt_stat;


//---------------------------------------------------------
// Constructor()

WebSocket::WebSocket()
{
  msg_recebida = "";
  m_webSocket = nullptr;
  m_clientConnected = false;
  initializeMessageHandlers();
}

// Implement the new methods
void WebSocket::SetWebSocket(ix::WebSocket* ws)
{
  m_webSocket = ws;
  m_clientConnected = (ws != nullptr);
}

bool WebSocket::IsClientConnected() const
{
  return m_clientConnected;
}

void WebSocket::SendMessage(const std::string& message, bool binary)
{
  if (m_webSocket != nullptr)
  {
    m_webSocket->send(message, binary);
    // You could add logging here if needed
    // reportEvent("Message sent: " + message);
  }
  else
  {
    reportRunWarning("Attempted to send message but WebSocket is not connected");
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
    else if(key == "IVPHELM_STATE")
      ivphelm_state = msg.GetString();
    else if(key == "WPT_INDEX")
      wpt_index = msg.GetDouble();
    else if(key == "IVPHELM_ALLSTOP")
      ivphelm_allstop = msg.GetString();
    else if(key == "WPT_STAT")
      wpt_stat = msg.GetString();
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
  // Aqui você pode incluir outras operações se necessário

  //Analysis the variable coming from Freeboard
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


  // If you have pending messages to send
  //Send messages to the websocket in FreeboardSK
  //SendMessage("NAV_LAT=50.0"); // Exemplo de envio de mensagem
  //SendMessage("NAV_HEADING=" + std::to_string(nav_heading)); // Sends atual heading
  //For some reason, is just sending from this on
  //I put the value_setpoint to facilitate the calibration
  //SendMessage("DESIRED_HEADING=" + std::to_string(value_setpoint)+"/NAV_HEADING="+std::to_string(nav_heading)); // Sends desired heading
  SendMessage("IVPHELM_STATE=" + ivphelm_state); // Sends the state of the IVP Helm
  SendMessage("WPT_INDEX=" + std::to_string(wpt_index)); // Sends the way point index
  SendMessage("IVPHELM_ALLSTOP=" + ivphelm_allstop); // Sends the all stop state
  SendMessage("WPT_STAT=" + wpt_stat); // Sends the way point state
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
  actab << "Mensagem Recebida"  << msg_recebida;
  // Caso futuramente você tenha acesso dinâmico, por exemplo,
  // ao número de clientes conectados, adicione:
  // actab << "Clients" << std::to_string(numeroDeClientes);

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
    Notify("DESIRED_RUDDER", "0");
    Notify("DESIRED_THRUST", "0");
    Notify("DESIRED_GEAR", "N");
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
  if (value == "true") {
    Notify("WAYPOINT", "true");
    Notify("MOOS_MANUAL_OVERIDE", "false");
    Notify("DESIRED_GEAR", "A");
  } 
}

void WebSocket::handleAutonomousStop(const std::string& value)
{
  if (value == "true") {
    Notify("MOOS_MANUAL_OVERRIDE", "true");
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

