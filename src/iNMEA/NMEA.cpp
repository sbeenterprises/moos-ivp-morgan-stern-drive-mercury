/************************************************************/
/*    NAME: DLima                                           */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: NMEA.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <string>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "MBUtils.h"
#include "ACTable.h"
#include "NMEA.h"

using namespace std;

// Variáveis globais (better to store in the class, but as given):
double nav_heading;  // Rumo atual do navio
double nav_speed;    // Velocidade atual do navio
double nav_lat;      // Latitude do navio
double nav_lon;      // Longitude do navio

//---------------------------------------------------------
// Constructor()

NMEA::NMEA()
{
  m_udp_sock = -1;
  m_socket_initialized = false;
}

//---------------------------------------------------------
// Destructor

NMEA::~NMEA()
{
  // Close socket if open
  if(m_udp_sock >= 0) {
    close(m_udp_sock);
  }
}

// Simple XOR Checksum
static std::string nmeaChecksum(const std::string &sentence_without_dollar)
{
  unsigned char xsum = 0;
  for (unsigned char c : sentence_without_dollar) {
    xsum ^= c;
  }

  std::ostringstream oss;
  oss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<int>(xsum);
  return oss.str();
}

static std::pair<std::string, char> decimalDegToNMEA(double decimal_deg, bool is_lat)
{
  // Convert absolute value to ddmm.mmmm or dddmm.mmmm
  double abs_val = std::fabs(decimal_deg);
  int degrees = (int)abs_val;
  double leftover = abs_val - (double)degrees;
  double total_minutes = leftover * 60.0; 
  int whole_minutes = (int)total_minutes;
  double frac_minutes = total_minutes - whole_minutes;
  
  char buf[16];
  if(is_lat) { // ddmm.mmmm
    std::snprintf(buf, sizeof(buf),
                  "%02d%02d.%04d",
                  degrees,
                  whole_minutes,
                  (int)(frac_minutes * 10000.0));
  } else {     // dddmm.mmmm
    std::snprintf(buf, sizeof(buf),
                  "%03d%02d.%04d",
                  degrees,
                  whole_minutes,
                  (int)(frac_minutes * 10000.0));
  }

  // Hemisphere letter
  char hemi;
  if(is_lat)
    hemi = (decimal_deg >= 0) ? 'N' : 'S';
  else
    hemi = (decimal_deg >= 0) ? 'E' : 'W';

  return std::make_pair(std::string(buf), hemi);
}


// Build a minimal GPRMC sentence containing lat, lon, speed, heading, etc.
static std::string buildGPRMC(double lat, double lon, double speed_m_s, double heading_deg)
{
  auto lat_nmea = decimalDegToNMEA(lat, true);
  auto lon_nmea = decimalDegToNMEA(lon, false);

  // Convert speed from m/s to knots
  double speed_knots = speed_m_s * 1.94384;

  // For demonstration, placeholders for time and date
  // $GPRMC,hhmmss.sss,A,ddmm.mmmm,N,dddmm.mmmm,W,speed,heading,DDMMYY,,*CS
  ostringstream oss;
  oss << "GPRMC,"
      << "000000.00,"   // Time (hhmmss.sss)
      << "A,"           // Status: A = Active
      << lat_nmea.first << "," << lat_nmea.second << ","
      << lon_nmea.first << "," << lon_nmea.second << ","
      << fixed << setprecision(2) << speed_knots << ","
      << fixed << setprecision(1) << heading_deg << ","
      << "010170" << ","  // Date (DDMMYY)
      << ",";             // No mag variation

  // Now add the checksum
  string sentence_no_dollar = oss.str();
  string checksum = nmeaChecksum(sentence_no_dollar);

  ostringstream full;
  full << "$" << sentence_no_dollar << "*" << checksum << "\r\n";
  return full.str();
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool NMEA::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    if(key == "NAV_HEADING")
      nav_heading = msg.GetDouble();
    else if(key == "NAV_SPEED")
      nav_speed = msg.GetDouble();
    else if(key == "NAV_LAT")
      nav_lat = msg.GetDouble();
    else if(key == "NAV_LONG")
      nav_lon = msg.GetDouble();
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }
  
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool NMEA::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool NMEA::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // -------------------------
  // Build NMEA string
  // -------------------------
  // For example, build just a single GPRMC sentence:
  std::string gprmc = buildGPRMC(nav_lat, nav_lon, nav_speed, nav_heading);

  // -------------------------
  // Send via UDP socket
  // -------------------------
  if(m_socket_initialized) {
    sendto(m_udp_sock,
           gprmc.c_str(),
           gprmc.size(),
           0,
           (struct sockaddr*)&m_server_addr,
           sizeof(m_server_addr));
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool NMEA::OnStartUp()
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
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // --------------------------------------------
  // Initialize the UDP socket here (port 10112)
  // --------------------------------------------
  m_udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if(m_udp_sock < 0) {
    reportConfigWarning("Failed to create UDP socket.");
    m_socket_initialized = false;
  } else {
    memset(&m_server_addr, 0, sizeof(m_server_addr));
    m_server_addr.sin_family = AF_INET;
    m_server_addr.sin_port   = htons(10112); //port here

    // For localhost, use "127.0.0.1" or read an IP from config if desired:
    m_server_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); //adress here

    m_socket_initialized = true;
  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void NMEA::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_HEADING", 0);
  Register("NAV_LAT", 0);
  Register("NAV_LONG", 0);
  Register("NAV_SPEED", 0);
  // ...
}

//------------------------------------------------------------
// Procedure: buildReport()

bool NMEA::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: NMEA Example                         " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Variable | Value |  | ";
  actab.addHeaderLines();
  actab << "NAV_HEADING" << nav_heading << "" << "";
  actab << "NAV_SPEED"   << nav_speed   << "" << "";
  actab << "NAV_LAT"     << nav_lat     << "" << "";
  actab << "NAV_LON"     << nav_lon     << "" << "";
  m_msgs << actab.getFormattedString();

  return(true);
}
