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

  if (key == "CONTROLE_MANUAL") {
    if (value == "true") {
      // Notify the MOOS community that the manual control is activated
      //Notify("DEPLOY", "true");
      Notify("MOOS_MANUAL_OVERIDE", "true");
    } else if (value == "false") {
      // Notify the MOOS community that the manual control is deactivated
      Notify("MOOS_MANUAL_OVERIDE", "false");
      // Notify the MOOS community with the desired thrust and rudder values
      Notify("DESIRED_RUDDER", "0");
      Notify("DESIRED_THRUST", "0");
      Notify("DESIRED_GEAR", "N");
    }
  } else if (key == "AUTONOMOUS_CONTROL") {
      // Notify the MOOS community that the autonomous control is activated
      Notify("MOOS_MANUAL_OVERIDE", "true");
      // Notify the MOOS community that the autonomous control is deactivated
      //Notify("MOOS_MANUAL_OVERIDE", "false");
      //Notify("DEPLOY", "true");
  } else if (key == "DESIRED_HEADING") {
    // Notify the MOOS community with the desired heading value
    double value_heading = std::stod(value);
    //Send to pHelmIvP instead of directly to the control
    Notify("DESIRED_HEADING", value_heading);
  } else if (key == "DESIRED_SPEED") {
    // Notify the MOOS community with the desired speed value
    double value_speed = std::stod(value);
    Notify("WPT_UPDATE", "speed=" + std::to_string(value_speed));
    //Notify("DESIRED_SPEED", value_speed);
  } else if (key == "DESIRED_RUDDER") {
    // Notify the MOOS community with the desired rudder value
    double value_rudder = std::stod(value);
    Notify("DESIRED_RUDDER", value_rudder);
  } else if (key == "DESIRED_THRUST") {
    //Notify the MOOS community with the desired thrust value
    double value_thrust = std::stod(value);
    Notify("DESIRED_THRUST", value_thrust);

  } else if (key == "DESIRED_GEAR") {
    //Notify the MOOS community with the desired gear value
    if (value == "1") {
      Notify("DESIRED_GEAR", "A");
    } else if (value == "0") {
      Notify("DESIRED_GEAR", "N");
    } else if (value == "-1") {
      Notify("DESIRED_GEAR", "R");
    } 
  } else if (key == "AUTONOMOUS_WAYPOINTS") {
    // Morgan City origin (update if the chart changes)
    double lat_origin = 29.71970895316288;
    double lon_origin = -91.14705165281887;
    
    // Initialize the geodesy conversion using the origin.
    m_geodesy.Initialise(lat_origin, lon_origin);

    // Container for each coordinate token after splitting by colon.
    std::vector<std::string> coordTokens;
    
    // Step 1: Split the input string by colon ':' to separate coordinate pairs.
    std::istringstream tokenStream(value);
    std::string token;
    while (std::getline(tokenStream, token, ':')) {
        if (!token.empty()) {
            coordTokens.push_back(token);
        }
    }
    
    // Vector to hold the final converted coordinate strings.
    std::vector<std::string> convertedCoords;
    
    // Process each coordinate token.
    for (const auto &coordStr : coordTokens) {
        // Step 2: Split the coordinate string by comma ','.
        size_t commaPos = coordStr.find(',');
        if (commaPos == std::string::npos) {
            std::cerr << "Invalid coordinate format for: " << coordStr << std::endl;
            continue;  // or handle error as needed
        }
        
        std::string latPart = coordStr.substr(0, commaPos);
        std::string lonPart = coordStr.substr(commaPos + 1);
        
        try {
            // Convert string parts to double.
            double orig_lon = std::stod(lonPart);
            double orig_lat = std::stod(latPart);
            
            // Step 3: Convert geographic coordinates to local UTM.
            // Note: The conversion function expects latitude first then longitude.
            double local_y = 0.0, local_x = 0.0;
            m_geodesy.LatLong2LocalUTM(orig_lat, orig_lon, local_y, local_x);
            
            // Step 4: Format the local coordinates (local_x as "lon", local_y as "lat")
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << local_x << "," << local_y;
            convertedCoords.push_back(oss.str());
        } catch (const std::exception &e) {
            std::cerr << "Error converting coordinate (" << coordStr << "): " << e.what() << std::endl;
        }
    }
    
    // Step 5: Construct the final string with the format:
    // "polygon=lon,lat:lon,lat:lon,lat:..."
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
    
    // Step 6: Publish to MOOSDB.
    Notify("WPT_UPDATE", wpt_update_str);

  } else if (key == "AUTONOMOUS_START") {
    if (value == "true") {
      Notify("WAYPOINT", "true");
      Notify("MOOS_MANUAL_OVERIDE", "false");
      //Notify("RETURN", "false");
      //Send gear commands so the boat can start
      Notify("DESIRED_GEAR", "A");
    } 
  } else if (key == "AUTONOMOUS_STOP") {
    if (value == "true") {
      //Notify("WAYPOINT", "false");
      Notify("MOOS_MANUAL_OVERRIDE", "true");
      //Notify("DESIRED_RUDDER", "0"); // Stop rudder
      //Notify("DESIRED_THRUST", "0"); // Stop thrust
      //Notify("DESIRED_GEAR", "N");
    }
  } else if (key == "AUTONOMOUS_RETURN_TO_BASE") {
    if (value == "true") {
      Notify("RETURN", "true");
    } else if (value == "false") {
      Notify("RETURN", "false");
    }
  } else if (key == "RETURN_UPDATE") {
    // 1) Setup origin (update these if chart changes)
    static constexpr double kLatOrigin = 29.71970895316288;
    static constexpr double kLonOrigin = -91.14705165281887;
    m_geodesy.Initialise(kLatOrigin, kLonOrigin);

    // 2) Split off the payload after the '='
    auto eqPos = value.find('=');
    if (eqPos == std::string::npos) {
        MOOSTrace("RETURN_UPDATE malformed: missing '=' in \"%s\"", value.c_str());
    }
    std::string payload = value.substr(eqPos + 1);

    // 3) Split lat/lon by comma
    auto commaPos = payload.find(',');
    if (commaPos == std::string::npos) {
        MOOSTrace("RETURN_UPDATE malformed: missing ',' in \"%s\"", payload.c_str());
    }
    std::string latStr = payload.substr(0, commaPos);
    std::string lonStr = payload.substr(commaPos + 1);

    // 4) Convert strings to doubles, with exception safety
    double latDeg = 0.0, lonDeg = 0.0;
    try {
        latDeg = std::stod(latStr);
        lonDeg = std::stod(lonStr);
    }
    catch (const std::invalid_argument& e) {
        MOOSTrace("RETURN_UPDATE parse error: invalid number \"%s\" or \"%s\"", latStr.c_str(), lonStr.c_str());

    }
    catch (const std::out_of_range& e) {
        MOOSTrace("RETURN_UPDATE parse error: value out of range \"%s\" or \"%s\"", latStr.c_str(), lonStr.c_str());
        
    }

    // 5) Validate ranges
    if (latDeg < -90.0 || latDeg > 90.0 || lonDeg < -180.0 || lonDeg > 180.0) {
        MOOSTrace("RETURN_UPDATE out‑of‑range coords: lat=%f lon=%f", latDeg, lonDeg);
        
    }

    // 6) Convert to local UTM
    double localY = 0.0, localX = 0.0;
    m_geodesy.LatLong2LocalUTM(latDeg, lonDeg, localY, localX);

    // 7) Build and publish back to MOOSDB
    std::ostringstream oss;
    oss << "points="
        << std::fixed << std::setprecision(1)
        << localX << "," << localY;
    Notify("RETURN_UPDATE", oss.str());
} else if (key == "STOP_CAN") {
    if (value == "true") {
      Notify("STOP_CAN", "true");
    } else if (value == "false") {
      Notify("STOP_CAN", "false");
    }
  } else if (key == "CONSTANT_HEADING") {
    if (value == "true") {
      Notify("CONSTANT_HEADING", "true");
    } else if (value == "false") {
      Notify("CONSTANT_HEADING", "false");
    }
  } else if (key == "SETPOINT_HEADING") {
    // Notify the MOOS community with the setpoint heading value
    value_setpoint = std::stod(value);
    Notify("SETPOINT_HEADING", value_setpoint);
  } else if (key == "HEADING_KP") {
    // Notify the MOOS community with the heading KP value
    double value_heading_kp = std::stod(value);
    Notify("HEADING_KP", value_heading_kp);
  } else if (key == "HEADING_KI") {
    // Notify the MOOS community with the heading KI value
    double value_heading_ki = std::stod(value);
    Notify("HEADING_KI", value_heading_ki);
  } else if (key == "HEADING_KD") {
    // Notify the MOOS community with the heading KD value
    double value_heading_kd = std::stod(value);
    Notify("HEADING_KD", value_heading_kd);
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

