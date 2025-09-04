/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: CAN.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <cstring>
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
     else if(key == "DESIRED_THRUST")
       m_desired_thrust = msg.GetDouble();
     else if(key == "DESIRED_RUDDER") {
       m_desired_rudder = msg.GetDouble();
       sendRudderCommand(m_desired_rudder);
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
  // Do your thing here!
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
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
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
}


//------------------------------------------------------------
// Procedure: buildReport()

bool CAN::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
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
  frame.can_id = 0x18F30A1A;
  frame.can_dlc = 8;
  
  sscanf(frame_str.substr(9).c_str(), "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx",
         &frame.data[0], &frame.data[1], &frame.data[2], &frame.data[3],
         &frame.data[4], &frame.data[5], &frame.data[6], &frame.data[7]);

  int bytes_sent = write(m_can_socket, &frame, sizeof(struct can_frame));
  
  if(bytes_sent < 0) {
    reportRunWarning("Failed to send CAN frame");
  }
}




