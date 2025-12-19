/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RadarTargets.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <cmath>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <cstring>
#include "MBUtils.h"
#include "ACTable.h"
#include "RadarTargets.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

RadarTargets::RadarTargets()
{
  m_udp_port = 8091;
  m_udp_host = "127.0.0.1";
  m_socket_fd = -1;
  m_own_lat = 0.0;
  m_own_lon = 0.0;
  m_own_x = 0.0;
  m_own_y = 0.0;
}

//---------------------------------------------------------
// Destructor

RadarTargets::~RadarTargets()
{
  if(m_socket_fd >= 0) {
    close(m_socket_fd);
  }
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool RadarTargets::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    if(key == "NAV_LAT") {
      m_own_lat = msg.GetDouble();
    }
    else if(key == "NAV_LON") {
      m_own_lon = msg.GetDouble();
    }
    else if(key == "NAV_X") {
      m_own_x = msg.GetDouble();
    }
    else if(key == "NAV_Y") {
      m_own_y = msg.GetDouble();
    }
    else if(key != "APPCAST_REQ") {
      reportRunWarning("Unhandled Mail: " + key);
    }
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool RadarTargets::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

/* bool RadarTargets::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if(m_socket_fd < 0) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // Receive UDP data (non-blocking)
  char buffer[1024];
  struct sockaddr_in sender_addr;
  socklen_t sender_len = sizeof(sender_addr);

  ssize_t bytes_received = recvfrom(m_socket_fd, buffer, sizeof(buffer)-1,
                                     MSG_DONTWAIT,
                                     (struct sockaddr*)&sender_addr,
                                     &sender_len);

  if(bytes_received > 0) {
    buffer[bytes_received] = '\0';
    string received_data(buffer);

    // Parse NMEA sentences (may contain multiple sentences)
    stringstream ss(received_data);
    string line;
    while(getline(ss, line)) {
      // Remove trailing whitespace
      while(!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
        line.pop_back();
      }

      if(!line.empty() && line[0] == '$') {
        parseRATTM(line);
      }
    }
  }

  AppCastingMOOSApp::PostReport();
  return(true);
} */

bool RadarTargets::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if(m_socket_fd < 0) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // Receive UDP data (non-blocking)
  char buffer[1024];
  struct sockaddr_in sender_addr;
  socklen_t sender_len = sizeof(sender_addr);

  ssize_t bytes_received = recvfrom(m_socket_fd, buffer, sizeof(buffer)-1,
                                     MSG_DONTWAIT,
                                     (struct sockaddr*)&sender_addr,
                                     &sender_len);

  if(bytes_received > 0) {
    buffer[bytes_received] = '\0';
    string received_data(buffer);

    stringstream ss(received_data);
    string line;
    while(getline(ss, line)) {
      while(!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
        line.pop_back();
      }

      if(!line.empty() && line[0] == '$') {
        parseRATTM(line);
      }
    }
  }

  // ========================================
  // NEW: Publish batched RADAR_TARGETS
  // ========================================
  publishRadarTargets();

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool RadarTargets::OnStartUp()
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
    if(param == "udp_port") {
      m_udp_port = atoi(value.c_str());
      handled = true;
    }
    else if(param == "udp_host") {
      m_udp_host = value;
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Setup UDP socket
  if(!setupUDPSocket()) {
    reportConfigWarning("Failed to setup UDP socket");
  }

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void RadarTargets::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_LAT", 0);
  Register("NAV_LON", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool RadarTargets::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "Radar Targets Status                        " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "UDP Port: " << m_udp_port << endl;
  m_msgs << "Socket FD: " << m_socket_fd << endl;
  m_msgs << "Own Position: (" << m_own_lat << ", " << m_own_lon << ")" << endl;
  m_msgs << "Tracked Targets: " << m_last_report_time.size() << endl;

  return(true);
}

//---------------------------------------------------------
// Procedure: setupUDPSocket()

bool RadarTargets::setupUDPSocket()
{
  m_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if(m_socket_fd < 0) {
    reportRunWarning("Failed to create UDP socket");
    return false;
  }

  // Set socket to non-blocking
  int flags = fcntl(m_socket_fd, F_GETFL, 0);
  fcntl(m_socket_fd, F_SETFL, flags | O_NONBLOCK);

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(m_udp_port);
  server_addr.sin_addr.s_addr = INADDR_ANY;

  if(bind(m_socket_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    reportRunWarning("Failed to bind UDP socket to port " + to_string(m_udp_port));
    close(m_socket_fd);
    m_socket_fd = -1;
    return false;
  }

  reportEvent("UDP socket bound to port " + to_string(m_udp_port));
  return true;
}

//---------------------------------------------------------
// Procedure: parseRATTM()

/* bool RadarTargets::parseRATTM(const string& sentence)
{
  // Expected format: $RATTM,26,5.76,133.8,T,22.87,232.7,R,5.7,2.3,N,,,T,,,M*22

  if(sentence.find("$RATTM") != 0 && sentence.find("$RATTM") != 0) {
    return false;
  }

  // Remove checksum if present
  string data = sentence;
  size_t checksum_pos = data.find('*');
  if(checksum_pos != string::npos) {
    data = data.substr(0, checksum_pos);
  }

  // Split by comma
  vector<string> fields;
  stringstream ss(data);
  string field;
  while(getline(ss, field, ',')) {
    fields.push_back(field);
  }

  // Need at least 11 fields (including sentence ID)
  if(fields.size() < 11) {
    reportRunWarning("Invalid RATTM sentence: insufficient fields");
    return false;
  }

  try {
    // Parse fields according to NMEA TTM format
    string target_number = fields[1];     // Field 1: Target number
    double range_nm = stod(fields[2]);    // Field 2: Distance (NM)
    double bearing_deg = stod(fields[3]); // Field 3: Bearing (degrees)
    string bearing_ref = fields[4];       // Field 4: T=true, R=relative
    double speed_kts = stod(fields[5]);   // Field 5: Speed (knots)
    double course_deg = stod(fields[6]);  // Field 6: Course (degrees)
    string course_ref = fields[7];        // Field 7: T=true, R=relative

    // Convert relative bearing to true bearing if needed
    // Note: You would need NAV_HEADING to convert relative to true
    // For now, assuming bearing is True (T)
    if(bearing_ref != "T") {
      reportRunWarning("Relative bearing not supported yet. Assuming true bearing.");
    }

    // Convert relative course to true course if needed
    if(course_ref != "T") {
      reportRunWarning("Relative course not supported yet. Assuming true course.");
    }

    // Convert range/bearing to local X,Y coordinates
    // Range is in nautical miles, convert to meters (1 NM = 1852 meters)
    double range_m = range_nm * 1852.0;

    // Convert bearing to radians (MOOS uses degrees from North, clockwise)
    double bearing_rad = bearing_deg * M_PI / 180.0;

    // Calculate target position in local coordinates relative to own ship
    // X = East, Y = North
    double delta_x = range_m * sin(bearing_rad);
    double delta_y = range_m * cos(bearing_rad);

    double target_x = m_own_x + delta_x;
    double target_y = m_own_y + delta_y;

    // Get current time
    double time = MOOSTime();

    // Build NODE_REPORT message in same format as pContactSpawn
    string node_report = "NAME=contato_" + target_number + ",";
    node_report += "TYPE=ship,";
    node_report += "TIME=" + doubleToString(time, 2) + ",";
    node_report += "X=" + doubleToString(target_x, 2) + ",";
    node_report += "Y=" + doubleToString(target_y, 2) + ",";
    node_report += "SPD=" + doubleToString(speed_kts, 2) + ",";
    node_report += "HDG=" + doubleToString(course_deg, 2) + ",";
    node_report += "YAW=" + doubleToString(course_deg, 2) + ",";
    node_report += "DEPTH=0.0,";
    node_report += "LENGTH=100,";
    node_report += "MODE=DRIVE";

    // Publish to MOOSDB
    Notify("NODE_REPORT", node_report);

    // Update last report time
    m_last_report_time[target_number] = time;

    reportEvent("Received RATTM target " + target_number +
                " at range " + to_string(range_nm) + " NM, bearing " +
                to_string(bearing_deg) + " deg");

  } catch(const exception& e) {
    reportRunWarning("Error parsing RATTM sentence: " + string(e.what()));
    return false;
  }

  return true;
} */

//---------------------------------------------------------
// Procedure: publishRadarTargets()
// Purpose: Batch publish all active radar targets for fusion

bool RadarTargets::parseRATTM(const string& sentence)
{
  // Expected format: $RATTM,26,5.76,133.8,T,22.87,232.7,R,5.7,2.3,N,,,T,,,M*22

  if(sentence.find("$RATTM") != 0) {
    return false;
  }

  // Remove checksum if present
  string data = sentence;
  size_t checksum_pos = data.find('*');
  if(checksum_pos != string::npos) {
    data = data.substr(0, checksum_pos);
  }

  // Split by comma
  vector<string> fields;
  stringstream ss(data);
  string field;
  while(getline(ss, field, ',')) {
    fields.push_back(field);
  }

  // Need at least 11 fields
  if(fields.size() < 11) {
    reportRunWarning("Invalid RATTM sentence: insufficient fields");
    return false;
  }

  try {
    // Parse fields
    string target_number = fields[1];
    double range_nm = stod(fields[2]);
    double bearing_deg = stod(fields[3]);
    string bearing_ref = fields[4];
    double speed_kts = stod(fields[5]);
    double course_deg = stod(fields[6]);
    string course_ref = fields[7];

    // Convert to meters
    double range_m = range_nm * 1852.0;
    double bearing_rad = bearing_deg * M_PI / 180.0;

    // Local position (relative to own ship)
    double delta_x = range_m * sin(bearing_rad);
    double delta_y = range_m * cos(bearing_rad);
    double target_x = m_own_x + delta_x;
    double target_y = m_own_y + delta_y;

    // Calculate velocity components
    double speed_ms = speed_kts * 0.514444;  // knots to m/s
    double course_rad = course_deg * M_PI / 180.0;
    double vx = speed_ms * sin(course_rad);
    double vy = speed_ms * cos(course_rad);

    double time = MOOSTime();

    // ========================================
    // Store in radar targets map (for RADAR_TARGETS)
    // ========================================
    RadarTarget target;
    target.x = target_x;
    target.y = target_y;
    target.vx = vx;
    target.vy = vy;
    target.range = range_m;
    target.bearing = bearing_deg;
    target.last_update = time;
    
    m_radar_targets[target_number] = target;

    // ========================================
    // Also publish NODE_REPORT (for backwards compatibility)
    // ========================================
    string node_report = "NAME=contato_" + target_number + ",";
    node_report += "TYPE=ship,";
    node_report += "TIME=" + doubleToString(time, 2) + ",";
    node_report += "X=" + doubleToString(target_x, 2) + ",";
    node_report += "Y=" + doubleToString(target_y, 2) + ",";
    node_report += "SPD=" + doubleToString(speed_kts, 2) + ",";
    node_report += "HDG=" + doubleToString(course_deg, 2) + ",";
    node_report += "YAW=" + doubleToString(course_deg, 2) + ",";
    node_report += "DEPTH=0.0,";
    node_report += "LENGTH=100,";
    node_report += "MODE=DRIVE";

    //Notify("NODE_REPORT", node_report);

    // Update tracking
    m_last_report_time[target_number] = time;

    reportEvent("Radar target " + target_number +
                " at (" + to_string(target_x) + "," + to_string(target_y) + ")");

  } catch(const exception& e) {
    reportRunWarning("Error parsing RATTM: " + string(e.what()));
    return false;
  }

  return true;
}

void RadarTargets::publishRadarTargets()
{
    if(m_radar_targets.empty()) return;
    
    // Build RADAR_TARGETS message (semicolon-separated)
    ostringstream oss;
    bool first = true;
    
    double current_time = MOOSTime();
    
    for(auto& pair : m_radar_targets) {
        const string& target_id = pair.first;
        const RadarTarget& target = pair.second;
        
        // Skip stale targets (older than 5 seconds)
        if(current_time - target.last_update > 5.0) continue;
        
        if(!first) oss << ";";
        first = false;
        
        oss << fixed << setprecision(2);
        oss << "x=" << target.x << ",";
        oss << "y=" << target.y << ",";
        oss << "vx=" << target.vx << ",";
        oss << "vy=" << target.vy << ",";
        oss << "range=" << target.range << ",";
        oss << "bearing=" << target.bearing << ",";
        oss << "label=radar_" << target_id;
    }
    
    if(!first) {  // At least one target
        Notify("RADAR_TARGETS", oss.str());
    }
}



