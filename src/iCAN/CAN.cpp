/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: CAN.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <cstring>
#include <thread>
#include <chrono>
#include "MBUtils.h"
#include "ACTable.h"
#include "CAN.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

CAN::CAN()
{
  m_desired_rudder = 0.0;
  m_desired_thrust = 0.0;
  m_commanded_thrust = 0.0;
  m_thrust_slew_rate = 20.0;   // % per second (ramp-up limit)
  m_last_thrust_time = 0.0;
  m_current_trim = 0.0;
  m_can_send = "false";  // Disable CAN sending by default
  m_can_socket = -1;
}

//---------------------------------------------------------
// Destructor

CAN::~CAN()
{
  if(m_can_socket != -1) {
    close(m_can_socket);
  }
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool CAN::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
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

     if(key == "FOO") 
       cout << "great!";
     else if(key == "DESIRED_THRUST") {
       // Only update the target; Iterate() ramps the commanded thrust
       // toward it so speed-ups are smooth and steady.
       m_desired_thrust = msg.GetDouble();
     }
     else if(key == "DESIRED_RUDDER") {
       m_desired_rudder = msg.GetDouble();
       sendRudderCommand(m_desired_rudder);
     }
     else if(key == "TRIM") {
       double trim_value = msg.GetDouble();
       //handleTrimCommand(trim_value); // --- TRIM DISABLED ---
     }
     else if(key == "CAN_SEND") {
       // Support both string ("true"/"false") and boolean formats
       m_can_send = msg.GetString();
     }
     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool CAN::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool CAN::Iterate()
{
  AppCastingMOOSApp::Iterate();

  sendRudderCommand(m_desired_rudder);

  // Slew-rate limit the thrust: ramp up gradually toward the target, but
  // allow immediate decreases (e.g. for collision avoidance / stop).
  double now = MOOSTime();
  double dt  = (m_last_thrust_time > 0.0) ? (now - m_last_thrust_time) : 0.0;
  m_last_thrust_time = now;

  double rise = m_desired_thrust - m_commanded_thrust;
  double max_step = m_thrust_slew_rate * dt;
  if(rise > max_step)
    m_commanded_thrust += max_step;   // rising faster than allowed: clamp the rate
  else
    m_commanded_thrust = m_desired_thrust;  // within step, or decreasing: apply now

  sendThrustCommand(m_commanded_thrust);

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool CAN::OnStartUp()
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
    if(param == "thrust_slew_rate") {
      double rate = atof(value.c_str());
      if(rate > 0.0)
        m_thrust_slew_rate = rate;
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  if(!initCAN()) {
    reportConfigWarning("Failed to initialize CAN interface");
    return(false);
  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void CAN::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("DESIRED_RUDDER", 0);
  Register("DESIRED_THRUST", 0);
  Register("TRIM", 0);
  Register("CAN_SEND", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool CAN::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File: iCAN - CAN Bus Interface             " << endl;
  m_msgs << "============================================" << endl;

  // CAN Interface Status
  string can_status = (m_can_socket != -1) ? "CONNECTED" : "DISCONNECTED";
  m_msgs << "CAN Interface (slcan0): " << can_status << endl;
  m_msgs << "CAN Socket ID: " << m_can_socket << endl;
  m_msgs << endl;

  // Current Values
  m_msgs << "Current Rudder Command: " << m_desired_rudder << "%" << endl;
  m_msgs << "Target Thrust: " << m_desired_thrust << "%" << endl;
  m_msgs << "Commanded Thrust (ramped): " << m_commanded_thrust << "%"
         << "  [slew " << m_thrust_slew_rate << " %/s]" << endl;
  m_msgs << "Current TRIM Value: " << m_current_trim << endl;
  m_msgs << endl;

  // CAN Frame Information
  string current_rudder_frame = getRudderCANFrame(m_desired_rudder);
  m_msgs << "Current Rudder CAN Frame: " << current_rudder_frame << endl;
  
  string direction = "CENTER";
  if(m_desired_rudder < 0) direction = "LEFT";
  else if(m_desired_rudder > 0) direction = "RIGHT";
  
  m_msgs << "Rudder Direction: " << direction << endl;
  m_msgs << "Rudder Percentage: " << abs((int)m_desired_rudder) << "%" << endl;
  m_msgs << endl;

  // Thrust Information (reflects the actually-sent, ramped value)
  string gear_command = getGearCommand(m_commanded_thrust);
  string thrust_state = (m_commanded_thrust > 0) ? "FORWARD" : "NEUTRAL";
  m_msgs << "Current Gear Command: " << gear_command << endl;
  m_msgs << "Thrust State: " << thrust_state << endl;

  if(m_commanded_thrust > 0) {
    string current_thrust_frame = getThrustCANFrame(m_commanded_thrust);
    m_msgs << "Current Thrust CAN Frame: " << current_thrust_frame << endl;
  }
  m_msgs << "Thrust Percentage: " << ((int)m_commanded_thrust) << "%" << endl;
  m_msgs << endl;

  // Statistics Table
  ACTable actab(2);
  actab << "Parameter | Value";
  actab.addHeaderLines();
  actab << "CAN ID" << "0x18F30A1A";
  actab << "Data Length" << "8 bytes";
  actab << "Update Rate" << "100ms (10Hz)";
  actab << "Interface" << "slcan0";
  m_msgs << actab.getFormattedString();

  return(true);
}

//---------------------------------------------------------
// Procedure: initCAN()

bool CAN::initCAN()
{
  m_can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(m_can_socket < 0) {
    return false;
  }

  strcpy(m_ifr.ifr_name, "slcan0");
  ioctl(m_can_socket, SIOCGIFINDEX, &m_ifr);

  m_can_addr.can_family = AF_CAN;
  m_can_addr.can_ifindex = m_ifr.ifr_ifindex;

  if(bind(m_can_socket, (struct sockaddr *)&m_can_addr, sizeof(m_can_addr)) < 0) {
    close(m_can_socket);
    m_can_socket = -1;
    return false;
  }

  return true;
}

//---------------------------------------------------------
// Procedure: getRudderCANFrame()

string CAN::getRudderCANFrame(double rudder_value)
{
  if(rudder_value == 0) {
    return "18F30A1A#0000FFFFFF1AFD";
  }
  
  if(rudder_value < 0) {
    int percent = (int)(-rudder_value);
    if(percent > 100) percent = 100;
    
    string left_frames[] = {
      "18F30A1A#FF9DFFFFFFFFE5", "18F30A1A#FF3AFFFFFFFFE5", "18F30A1A#FED7FFFFFFFFE5", "18F30A1A#FE74FFFFFFFFE5", "18F30A1A#FE11FFFFFFFFE5",
      "18F30A1A#FDAEFFFFFFFFE5", "18F30A1A#FD4BFFFFFFFFE5", "18F30A1A#FCE8FFFFFFFFE5", "18F30A1A#FC85FFFFFFFFE5", "18F30A1A#FC22FFFFFFFFE5",
      "18F30A1A#FBBFFFFFFFFFE5", "18F30A1A#FB5CFFFFFFFFE5", "18F30A1A#FAF9FFFFFFFFE5", "18F30A1A#FA96FFFFFFFFE5", "18F30A1A#FA33FFFFFFFFE5",
      "18F30A1A#F9D0FFFFFFFFE5", "18F30A1A#F96DFFFFFFFFE5", "18F30A1A#F90AFFFFFFFFE5", "18F30A1A#F8A7FFFFFFFFE5", "18F30A1A#F844FFFFFFFFE5",
      "18F30A1A#F7E1FFFFFFFFE5", "18F30A1A#F77EFFFFFFFFE5", "18F30A1A#F71BFFFFFFFFE5", "18F30A1A#F6B8FFFFFFFFE5", "18F30A1A#F655FFFFFFFFE5",
      "18F30A1A#F5F2FFFFFFFFE5", "18F30A1A#F58FFFFFFFFFE5", "18F30A1A#F52CFFFFFFFFE5", "18F30A1A#F4C9FFFFFFFFE5", "18F30A1A#F466FFFFFFFFE5",
      "18F30A1A#F403FFFFFFFFE5", "18F30A1A#F3A0FFFFFFFFE5", "18F30A1A#F33DFFFFFFFFE5", "18F30A1A#F2DAFFFFFFFFE5", "18F30A1A#F277FFFFFFFFE5",
      "18F30A1A#F214FFFFFFFFE5", "18F30A1A#F1B1FFFFFFFFE5", "18F30A1A#F14EFFFFFFFFE5", "18F30A1A#F0EBFFFFFFFFE5", "18F30A1A#F088FFFFFFFFE5",
      "18F30A1A#F025FFFFFFFFE5", "18F30A1A#EFC2FFFFFFFFE5", "18F30A1A#EF5FFFFFFFFFE5", "18F30A1A#EEFCFFFFFFFFE5", "18F30A1A#EE99FFFFFFFFE5",
      "18F30A1A#EE36FFFFFFFFE5", "18F30A1A#EDD3FFFFFFFFE5", "18F30A1A#ED70FFFFFFFFE5", "18F30A1A#ED0DFFFFFFFFE5", "18F30A1A#ECAAFFFFFFFFE5",
      "18F30A1A#EC47FFFFFFFFE5", "18F30A1A#EBE4FFFFFFFFE5", "18F30A1A#EB81FFFFFFFFE5", "18F30A1A#EB1EFFFFFFFFE5", "18F30A1A#EABBFFFFFFFFE5",
      "18F30A1A#EA58FFFFFFFFE5", "18F30A1A#E9F5FFFFFFFFE5", "18F30A1A#E992FFFFFFFFE5", "18F30A1A#E92FFFFFFFFFE5", "18F30A1A#E8CCFFFFFFFFE5",
      "18F30A1A#E869FFFFFFFFE5", "18F30A1A#E806FFFFFFFFE5", "18F30A1A#E7A3FFFFFFFFE5", "18F30A1A#E740FFFFFFFFE5", "18F30A1A#E6DDFFFFFFFFE5",
      "18F30A1A#E67AFFFFFFFFE5", "18F30A1A#E617FFFFFFFFE5", "18F30A1A#E5B4FFFFFFFFE5", "18F30A1A#E551FFFFFFFFE5", "18F30A1A#E4EEFFFFFFFFE5",
      "18F30A1A#E48BFFFFFFFFE5", "18F30A1A#E428FFFFFFFFE5", "18F30A1A#E3C5FFFFFFFFE5", "18F30A1A#E362FFFFFFFFE5", "18F30A1A#E2FFFFFFFFFFE5",
      "18F30A1A#E29CFFFFFFFFE5", "18F30A1A#E239FFFFFFFFE5", "18F30A1A#E1D6FFFFFFFFE5", "18F30A1A#E173FFFFFFFFE5", "18F30A1A#E110FFFFFFFFE5",
      "18F30A1A#E0ADFFFFFFFFE5", "18F30A1A#E04AFFFFFFFFE5", "18F30A1A#DFE7FFFFFFFFE5", "18F30A1A#DF84FFFFFFFFE5", "18F30A1A#DF21FFFFFFFFE5",
      "18F30A1A#DEBEFFFFFFFFE5", "18F30A1A#DE5BFFFFFFFFE5", "18F30A1A#DDF6FFFFFFFFE5", "18F30A1A#DD93FFFFFFFFE5", "18F30A1A#DD30FFFFFFFFE5",
      "18F30A1A#DCCDFFFFFFFFE5", "18F30A1A#DC6AFFFFFFFFE5", "18F30A1A#DC07FFFFFFFFE5", "18F30A1A#DBA4FFFFFFFFE5", "18F30A1A#DB41FFFFFFFFE5",
      "18F30A1A#DADEFFFFFFFFE5", "18F30A1A#DA7BFFFFFFFFE5", "18F30A1A#DA18FFFFFFFFE5", "18F30A1A#D9B5FFFFFFFFE5", "18F30A1A#D952FFFFFFFFE5"
    };
    
    if(percent >= 1 && percent <= 100) {
      return left_frames[percent-1];
    }
  } else {
    int percent = (int)rudder_value;
    if(percent > 100) percent = 100;
    
    string right_frames[] = {
      "18F30A1A#01B9FFFFFF1AFD", "18F30A1A#021AFFFFFF1AFD", "18F30A1A#027AFFFFFF1AFD", "18F30A1A#02DAFFFFFF1AFD", "18F30A1A#033BFFFFFF1AFD",
      "18F30A1A#039BFFFFFF1AFD", "18F30A1A#03FCFFFFFF1AFD", "18F30A1A#045CFFFFFF1AFD", "18F30A1A#04BCFFFFFF1AFD", "18F30A1A#051DFFFFFF1AFD",
      "18F30A1A#057DFFFFFF1AFD", "18F30A1A#05DDFFFFFF1AFD", "18F30A1A#063EFFFFFF1AFD", "18F30A1A#069EFFFFFF1AFD", "18F30A1A#06FEFFFFFF1AFD",
      "18F30A1A#075FFFFFFF1AFD", "18F30A1A#07BFFFFFFF1AFD", "18F30A1A#081FFFFFFF1AFD", "18F30A1A#0880FFFFFF1AFD", "18F30A1A#08E0FFFFFF1AFD",
      "18F30A1A#0941FFFFFF1AFD", "18F30A1A#09A1FFFFFF1AFD", "18F30A1A#0A01FFFFFF1AFD", "18F30A1A#0A62FFFFFF1AFD", "18F30A1A#0AC2FFFFFF1AFD",
      "18F30A1A#0B22FFFFFF1AFD", "18F30A1A#0B83FFFFFF1AFD", "18F30A1A#0BE3FFFFFF1AFD", "18F30A1A#0C43FFFFFF1AFD", "18F30A1A#0CA4FFFFFF1AFD",
      "18F30A1A#0D04FFFFFF1AFD", "18F30A1A#0D64FFFFFF1AFD", "18F30A1A#0DC5FFFFFF1AFD", "18F30A1A#0E25FFFFFF1AFD", "18F30A1A#0E86FFFFFF1AFD",
      "18F30A1A#0EE6FFFFFF1AFD", "18F30A1A#0F46FFFFFF1AFD", "18F30A1A#0FA7FFFFFF1AFD", "18F30A1A#1007FFFFFF1AFD", "18F30A1A#1067FFFFFF1AFD",
      "18F30A1A#10C8FFFFFF1AFD", "18F30A1A#1128FFFFFF1AFD", "18F30A1A#1189FFFFFF1AFD", "18F30A1A#11E9FFFFFF1AFD", "18F30A1A#1249FFFFFF1AFD",
      "18F30A1A#12AAFFFFFF1AFD", "18F30A1A#130AFFFFFF1AFD", "18F30A1A#136AFFFFFF1AFD", "18F30A1A#13CBFFFFFF1AFD", "18F30A1A#142BFFFFFF1AFD",
      "18F30A1A#148BFFFFFF1AFD", "18F30A1A#14ECFFFFFF1AFD", "18F30A1A#154CFFFFFF1AFD", "18F30A1A#15ACFFFFFF1AFD", "18F30A1A#160DFFFFFF1AFD",
      "18F30A1A#166DFFFFFF1AFD", "18F30A1A#16CEFFFFFF1AFD", "18F30A1A#172EFFFFFF1AFD", "18F30A1A#178EFFFFFF1AFD", "18F30A1A#17EFFFFFFF1AFD",
      "18F30A1A#184FFFFFFF1AFD", "18F30A1A#18AFFFFFFF1AFD", "18F30A1A#1910FFFFFF1AFD", "18F30A1A#1970FFFFFF1AFD", "18F30A1A#19D1FFFFFF1AFD",
      "18F30A1A#1A31FFFFFF1AFD", "18F30A1A#1A91FFFFFF1AFD", "18F30A1A#1AF2FFFFFF1AFD", "18F30A1A#1B52FFFFFF1AFD", "18F30A1A#1BB2FFFFFF1AFD",
      "18F30A1A#1C13FFFFFF1AFD", "18F30A1A#1C73FFFFFF1AFD", "18F30A1A#1CD4FFFFFF1AFD", "18F30A1A#1D34FFFFFF1AFD", "18F30A1A#1D94FFFFFF1AFD",
      "18F30A1A#1DF5FFFFFF1AFD", "18F30A1A#1E55FFFFFF1AFD", "18F30A1A#1EB5FFFFFF1AFD", "18F30A1A#1F16FFFFFF1AFD", "18F30A1A#1F76FFFFFF1AFD",
      "18F30A1A#1FD7FFFFFF1AFD", "18F30A1A#2037FFFFFF1AFD", "18F30A1A#2097FFFFFF1AFD", "18F30A1A#20F8FFFFFF1AFD", "18F30A1A#2158FFFFFF1AFD",
      "18F30A1A#21B8FFFFFF1AFD", "18F30A1A#2218FFFFFF1AFD", "18F30A1A#2279FFFFFF1AFD", "18F30A1A#22D9FFFFFF1AFD", "18F30A1A#2339FFFFFF1AFD",
      "18F30A1A#239AFFFFFF1AFD", "18F30A1A#23FAFFFFFF1AFD", "18F30A1A#245AFFFFFF1AFD", "18F30A1A#24BBFFFFFF1AFD", "18F30A1A#251BFFFFFF1AFD",
      "18F30A1A#257CFFFFFF1AFD", "18F30A1A#25DCFFFFFF1AFD", "18F30A1A#263CFFFFFF1AFD", "18F30A1A#269DFFFFFF1AFD", "18F30A1A#26FDFFFFFF1AFD"
    };
    
    if(percent >= 1 && percent <= 100) {
      return right_frames[percent-1];
    }
  }
  
  return "18F30A1A#0000FFFFFF1AFD";
}

//---------------------------------------------------------
// Procedure: sendRudderCommand()

void CAN::sendRudderCommand(double rudder_value)
{
  if(m_can_socket == -1) {
    return;
  }

  string frame_str = getRudderCANFrame(rudder_value);
  
  struct can_frame frame;
  frame.can_id = 0x18F30A1A | CAN_EFF_FLAG;
  frame.can_dlc = 8;

  // calculate DLC from payload size in string
  const char* hex = frame_str.c_str() + 9; // depois do '#'
  size_t nhex = strlen(hex);
  size_t nbytes = nhex / 2;
  if (nbytes > 8) nbytes = 8; // can_frame suporta até 8
  frame.can_dlc = static_cast<__u8>(nbytes);

  
  sscanf(frame_str.substr(9).c_str(), "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
         &frame.data[0], &frame.data[1], &frame.data[2], &frame.data[3],
         &frame.data[4], &frame.data[5], &frame.data[6], &frame.data[7]);

  
  // zero and parse
  memset(frame.data, 0, sizeof(frame.data));
  for (size_t i = 0; i < nbytes; ++i) {
    unsigned int byte_val = 0;
    sscanf(hex + 2*i, "%02x", &byte_val);
    frame.data[i] = static_cast<__u8>(byte_val);
  }

  if (m_can_send == "true") {
    int bytes_sent = write(m_can_socket, &frame, sizeof(struct can_frame));
    
    if(bytes_sent < 0) {
      reportRunWarning("Failed to send CAN frame");
    }
  }
  else {
    reportRunWarning("CAN_SEND is false, not sending CAN frame");
  }
}

//---------------------------------------------------------
// Procedure: sendThrustCommand()

void CAN::sendThrustCommand(double thrust_value)
{
  if(m_can_socket == -1) {
    return;
  }

  // Send gear command first
  string gear_frame_str = getGearCommand(thrust_value);
  struct can_frame gear_frame;
  gear_frame.can_id = 0x18F005D0 | CAN_EFF_FLAG;
  gear_frame.can_dlc = 8;

  sscanf(gear_frame_str.substr(9).c_str(), "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
         &gear_frame.data[0], &gear_frame.data[1], &gear_frame.data[2], &gear_frame.data[3],
         &gear_frame.data[4], &gear_frame.data[5], &gear_frame.data[6], &gear_frame.data[7]);

  if (m_can_send == "true") {
    int gear_bytes_sent = write(m_can_socket, &gear_frame, sizeof(struct can_frame));
    
    if(gear_bytes_sent < 0) {
      reportRunWarning("Failed to send gear CAN frame");
    }
  }
  else {
    reportRunWarning("CAN_SEND is false, not sending gear CAN frame");
  }

  // Always send thrust command (even if 0)
  string thrust_frame_str = getThrustCANFrame(thrust_value);
  struct can_frame thrust_frame;
  thrust_frame.can_id = 0x18000017 | CAN_EFF_FLAG;
  thrust_frame.can_dlc = 8;

  sscanf(thrust_frame_str.substr(9).c_str(), "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
         &thrust_frame.data[0], &thrust_frame.data[1], &thrust_frame.data[2], &thrust_frame.data[3],
         &thrust_frame.data[4], &thrust_frame.data[5], &thrust_frame.data[6], &thrust_frame.data[7]);

  if (m_can_send == "true") {
    int thrust_bytes_sent = write(m_can_socket, &thrust_frame, sizeof(struct can_frame));
    
    if(thrust_bytes_sent < 0) {
      reportRunWarning("Failed to send thrust CAN frame");
    }
  }
  else {
    reportRunWarning("CAN_SEND is false, not sending thrust CAN frame");
  }
}

//---------------------------------------------------------
// Procedure: getGearCommand()

std::string CAN::getGearCommand(double thrust_value)
{
  if(thrust_value > 0) {
    return "18F005D0#7EFFFFFFFFFFFFFF"; // Forward gear
  } else if (thrust_value == 0) {
    return "18F005D0#7DFFFFFFFFFFFFFF"; // Neutral gear
  } else {
    return "18F005D0#7CFFFFFFFFFFFFFF"; // Reverse Gear
  }
}

//---------------------------------------------------------
// Procedure: getThrustCANFrame()

std::string CAN::getThrustCANFrame(double thrust_value)
{
  //Removed here when thrust bellow zero
  //if(thrust_value <= 0) {
  //  return "18000017#C1E015E1FFFFFFFF"; // 0%
  //}

  // Convert thrust value to percentage (0-100)
  // Use absolute value to handle negative thrust (reverse)
  int percentage = static_cast<int>(std::abs(thrust_value));
  if(percentage > 100) percentage = 100;

  // Thrust command mapping based on percentage
  std::string thrust_commands[] = {
    "18000017#C1E015E1FFFFFFFF", // 0%
    "18000017#C1C016E1FFFFFFFF", // 1%
    "18000017#C11018E1FFFFFFFF", // 2%
    "18000017#C1F018E1FFFFFFFF", // 3%
    "18000017#C1401AE1FFFFFFFF", // 4%
    "18000017#C1201BE1FFFFFFFF", // 5%
    "18000017#C1701CE1FFFFFFFF", // 6%
    "18000017#C1501DE1FFFFFFFF", // 7%
    "18000017#C1A01EE1FFFFFFFF", // 8%
    "18000017#C1801FE1FFFFFFFF", // 9%
    "18000017#C1D020E1FFFFFFFF", // 10%
    "18000017#C1B021E1FFFFFFFF", // 11%
    "18000017#C10023E1FFFFFFFF", // 12%
    "18000017#C1E023E1FFFFFFFF", // 13%
    "18000017#C13025E1FFFFFFFF", // 14%
    "18000017#C11026E1FFFFFFFF", // 15%
    "18000017#C16027E1FFFFFFFF", // 16%
    "18000017#C14028E1FFFFFFFF", // 17%
    "18000017#C19029E1FFFFFFFF", // 18%
    "18000017#C1702AE1FFFFFFFF", // 19%
    "18000017#C1702AE1FFFFFFFF", // 20%
    "18000017#C1C02BE1FFFFFFFF", // 21%
    "18000017#C1F02DE1FFFFFFFF", // 22%
    "18000017#C15032E1FFFFFFFF", // 23%
    "18000017#C10031E1FFFFFFFF", // 24%
    "18000017#C10031E1FFFFFFFF", // 25%
    "18000017#C15032E1FFFFFFFF", // 26%
    "18000017#C15032E1FFFFFFFF", // 27%
    "18000017#C16035E1FFFFFFFF", // 28%
    "18000017#C16035E1FFFFFFFF", // 29%
    "18000017#C1B036E1FFFFFFFF", // 30%
    "18000017#C1E038E1FFFFFFFF", // 31%
    "18000017#C1E038E1FFFFFFFF", // 32%
    "18000017#C1C039E1FFFFFFFF", // 33%
    "18000017#C1C039E1FFFFFFFF", // 34%
    "18000017#C1F03BE1FFFFFFFF", // 35%
    "18000017#C1F03BE1FFFFFFFF", // 36%
    "18000017#C1403DE1FFFFFFFF", // 37%
    "18000017#C1203EE1FFFFFFFF", // 38%
    "18000017#C15040E1FFFFFFFF", // 39%
    "18000017#C15040E1FFFFFFFF", // 40%
    "18000017#C1A041E1FFFFFFFF", // 41%
    "18000017#C18042E1FFFFFFFF", // 42%
    "18000017#C1D043E1FFFFFFFF", // 43%
    "18000017#C1B044E1FFFFFFFF", // 44%
    "18000017#C10046E1FFFFFFFF", // 45%
    "18000017#C1E046E1FFFFFFFF", // 46%
    "18000017#C13048E1FFFFFFFF", // 47%
    "18000017#C11049E1FFFFFFFF", // 48%
    "18000017#C1604AE1FFFFFFFF", // 49%
    "18000017#C1404BE1FFFFFFFF", // 50%
    "18000017#C1904CE1FFFFFFFF", // 51%
    "18000017#C1C04EE1FFFFFFFF", // 52%
    "18000017#C1C04EE1FFFFFFFF", // 53%
    "18000017#C1C04EE1FFFFFFFF", // 54%
    "18000017#C1C04EE1FFFFFFFF", // 55% (missing in original, using 54%)
    "18000017#C1D051E1FFFFFFFF", // 56%
    "18000017#C12053E1FFFFFFFF", // 57%
    "18000017#C10054E1FFFFFFFF", // 58%
    "18000017#C15055E1FFFFFFFF", // 59%
    "18000017#C13056E1FFFFFFFF", // 60%
    "18000017#C16058E1FFFFFFFF", // 61%
    "18000017#C16058E1FFFFFFFF", // 62%
    "18000017#C1B059E1FFFFFFFF", // 63%
    "18000017#C1C05CE1FFFFFFFF", // 64%
    "18000017#C1C05CE1FFFFFFFF", // 65%
    "18000017#C1C05CE1FFFFFFFF", // 66%
    "18000017#C1C05CE1FFFFFFFF", // 67%
    "18000017#C1105EE1FFFFFFFF", // 68%
    "18000017#C1F05EE1FFFFFFFF", // 69%
    "18000017#C12061E1FFFFFFFF", // 70%
    "18000017#C1404BE1FFFFFFFF", // 71%
    "18000017#C15063E1FFFFFFFF", // 72%
    "18000017#C15063E1FFFFFFFF", // 73%
    "18000017#C18065E1FFFFFFFF", // 74%
    "18000017#C18065E1FFFFFFFF", // 75%
    "18000017#C1B067E1FFFFFFFF", // 76%
    "18000017#C1B067E1FFFFFFFF", // 77%
    "18000017#C10069E1FFFFFFFF", // 78%
    "18000017#C1E069E1FFFFFFFF", // 79%
    "18000017#C1106CE1FFFFFFFF", // 80%
    "18000017#C1906FE1FFFFFFFF", // 81%
    "18000017#C1906FE1FFFFFFFF", // 82%
    "18000017#C1906FE1FFFFFFFF", // 83%
    "18000017#C1906FE1FFFFFFFF", // 84%
    "18000017#C1C071E1FFFFFFFF", // 85%
    "18000017#C1C071E1FFFFFFFF", // 86%
    "18000017#C1F073E1FFFFFFFF", // 87%
    "18000017#C1F073E1FFFFFFFF", // 88%
    "18000017#C1D074E1FFFFFFFF", // 89%
    "18000017#C12076E1FFFFFFFF", // 90%
    "18000017#C15078E1FFFFFFFF", // 91%
    "18000017#C15078E1FFFFFFFF", // 92%
    "18000017#C13079E1FFFFFFFF", // 93%
    "18000017#C13079E1FFFFFFFF", // 94%
    "18000017#C1607BE1FFFFFFFF", // 95%
    "18000017#C1607BE1FFFFFFFF", // 96%
    "18000017#C14083E1FFFFFFFF", // 97%
    "18000017#C14083E1FFFFFFFF", // 98%
    "18000017#C14083E1FFFFFFFF", // 99%
    "18000017#C14083E1FFFFFFFF"  // 100%
  };

  return thrust_commands[percentage];
}

//---------------------------------------------------------
// Procedure: handleTrimCommand()

void CAN::handleTrimCommand(double trim_value)
{
  if(trim_value == 1.0) {
    m_current_trim = 1.0;
    // Execute trim sequence for TRIM=1 in background thread
    std::thread trim_thread(&CAN::executeTrimSequence, this, "0CFF08D0#F1FFFFFFFFFFFFFF");
    trim_thread.detach();
  }
  else if(trim_value == -1.0) {
    m_current_trim = -1.0;
    // Execute trim sequence for TRIM=-1 in background thread
    std::thread trim_thread(&CAN::executeTrimSequence, this, "0CFF08D0#F4FFFFFFFFFFFFFF");
    trim_thread.detach();
  }
}

//---------------------------------------------------------
// Procedure: executeTrimSequence()

void CAN::executeTrimSequence(const std::string& can_command)
{
  if(m_can_socket == -1) {
    reportRunWarning("CAN socket not initialized for trim sequence");
    return;
  }

  // Send the CAN command 12 times with 100ms intervals
  for(int i = 0; i < 12; i++) {
    // Parse CAN frame from command string
    struct can_frame frame;
    
    // Extract CAN ID and data from string format "ID#DATA"
    size_t hash_pos = can_command.find('#');
    if(hash_pos == std::string::npos) {
      reportRunWarning("Invalid CAN command format for trim sequence");
      return;
    }
    
    std::string id_str = can_command.substr(0, hash_pos);
    std::string data_str = can_command.substr(hash_pos + 1);
    
    // Convert hex string to CAN ID
    frame.can_id = std::stoul(id_str, nullptr, 16) | CAN_EFF_FLAG;
    frame.can_dlc = data_str.length() / 2;
    
    // Convert hex data string to bytes
    for(size_t j = 0; j < frame.can_dlc && j < 8; j++) {
      std::string byte_str = data_str.substr(j * 2, 2);
      frame.data[j] = static_cast<__u8>(std::stoul(byte_str, nullptr, 16));
    }
    
    if (m_can_send == "true") {
      // Send the frame
      int bytes_sent = write(m_can_socket, &frame, sizeof(struct can_frame));
      if(bytes_sent < 0) {
        reportRunWarning("Failed to send trim CAN frame " + std::to_string(i + 1));
      }
    }
    else {
      reportRunWarning("CAN_SEND is false, not sending trim CAN frame");
    }
    
    // Wait 100ms before next command (except for last iteration)
    if(i < 11) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  
  // Set TRIM back to 0 after sequence is complete
  m_current_trim = 0.0;
  Notify("TRIM", 0.0);
}

