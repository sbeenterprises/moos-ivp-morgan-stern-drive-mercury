/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: AisParser.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "AisParser.h"

// UDP includes
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

// libais includes
#include <memory>
#include <string>
#include "../iDivisorNMEA/libais/ais.h"
#include "../iDivisorNMEA/libais/ais.cpp"
#include "../iDivisorNMEA/libais/ais_bitset.cpp"
#include "../iDivisorNMEA/libais/ais1_2_3.cpp"

// Geodesy include
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "../MOOSGeodesy.cpp"

#define LOCAL_SERVER_PORT 10111
#define MAX_MSG 1000

using namespace std;

// UDP socket variables
int sd, rc, n;
socklen_t cliLen;
struct sockaddr_in cliAddr, servAddr;
char msg[MAX_MSG];

//---------------------------------------------------------
// Constructor()

AisParser::AisParser()
{
}

//---------------------------------------------------------
// Destructor

AisParser::~AisParser()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool AisParser::OnNewMail(MOOSMSG_LIST &NewMail)
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

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool AisParser::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool AisParser::Iterate()
{
  AppCastingMOOSApp::Iterate();

  memset(msg, 0x0, MAX_MSG);

  // Receive UDP message
  cliLen = sizeof(cliAddr);

  try {
    n = recvfrom(sd, msg, MAX_MSG, 0,
      (struct sockaddr *) &cliAddr, &cliLen);
  }
  catch (std::system_error& e) {
    std::cout << e.what();
  }

  if(n < 0) {
    printf(": cannot receive data \n");
  }

  printf(": from %s:UDP%u : %s \n",
    inet_ntoa(cliAddr.sin_addr),
    ntohs(cliAddr.sin_port), msg);

  std::string msg_string = msg;

  // Parse only AIS messages
  if (msg_string.substr(0, 6) == "!AIVDM") {
    Notify("AIS_RAW_MSG", msg_string);
    reportEvent("AIVDM received: " + msg_string);

    try {
      const std::string body(libais::GetBody(msg));
      const int pad = libais::GetPad(msg);

      if (pad >= 0) {
        reportEvent("Parsing AIS message - Pad valid: " + std::to_string(pad));

        std::unique_ptr<libais::Ais1_2_3> ais_msg(new libais::Ais1_2_3(body.c_str(), pad));

        std::string debug_info = "AIS Parse OK - MMSI:" + std::to_string(ais_msg->mmsi) +
                                 " SOG:" + std::to_string(ais_msg->sog) +
                                 " Lat:" + std::to_string(ais_msg->position.lat_deg) +
                                 " Lon:" + std::to_string(ais_msg->position.lng_deg);
        reportEvent(debug_info);

        // Skip own vessel MMSI 710400014
        if (ais_msg->mmsi != 710400014) {
          reportEvent("Publishing NODE_REPORT for MMSI: " + std::to_string(ais_msg->mmsi));

          Notify("MESSAGE_ID", ais_msg->message_id);
          Notify("MESSAGE_MMSI", ais_msg->mmsi);
          Notify("MESSAGE_NAVSTATUS", ais_msg->nav_status);
          Notify("MESSAGE_SOG", ais_msg->sog);
          Notify("MESSAGE_LONGITUDE", ais_msg->position.lng_deg);
          Notify("MESSAGE_LATITUDE", ais_msg->position.lat_deg);
          Notify("MESSAGE_TRUEHEADING", ais_msg->true_heading);

          std::string msg_lat = std::to_string(ais_msg->position.lat_deg);
          std::string msg_lon = std::to_string(ais_msg->position.lng_deg);
          std::string msg_spd = std::to_string(ais_msg->sog);
          std::string msg_cog = std::to_string(ais_msg->cog);
          std::string msg_mmsi = std::to_string(ais_msg->mmsi);

          double time = MOOSTime();
          std::string time_string = std::to_string(time);

          // Morgan City origin
          double lat_origin = 29.71970895316288;
          double lon_origin = -91.14705165281887;
          double nav_x = 0;
          double nav_y = 0;

          m_geodesy.Initialise(lat_origin, lon_origin);
          m_geodesy.LatLong2LocalGrid(ais_msg->position.lat_deg, ais_msg->position.lng_deg, nav_y, nav_x);

          std::string x = std::to_string(nav_x);
          std::string y = std::to_string(nav_y);

          std::string node_report = "NAME=CONTATO_TESTE,TYPE=SHIP,TIME=" + time_string +
                                   ",LAT=" + msg_lat + ",LON=" + msg_lon + ",SPD=" + msg_spd +
                                   ",HDG=" + msg_cog + ",LENGTH=100,MODE=DRIVE,X=" + x + ",Y=" + y;

          reportEvent("Publishing NODE_REPORT: " + node_report);
          Notify("NODE_REPORT", node_report);
        } else {
          reportEvent("MMSI ignored (own vessel): " + std::to_string(ais_msg->mmsi));
        }
      } else {
        reportRunWarning("Invalid pad (pad < 0), AIS message not processed");
      }
    }
    catch (std::system_error& e) {
      std::cout << e.what();
      reportRunWarning("AIS system_error: " + std::string(e.what()));
    }
    catch (std::exception& e) {
      reportRunWarning("AIS exception: " + std::string(e.what()));
    }
    catch (...) {
      reportRunWarning("Unknown error processing AIS message");
    }
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool AisParser::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  // Create UDP socket
  sd = socket(AF_INET, SOCK_DGRAM, 0);
  if(sd < 0) {
    printf("cannot open socket\n");
    exit(1);
  }

  // Bind to local port 10111
  servAddr.sin_family = AF_INET;
  servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servAddr.sin_port = htons(LOCAL_SERVER_PORT);
  rc = bind(sd, (struct sockaddr *) &servAddr, sizeof(servAddr));
  if(rc < 0) {
    printf("cannot bind port number %d\n", LOCAL_SERVER_PORT);
    exit(1);
  }

  printf("waiting for data on port UDP %u\n", LOCAL_SERVER_PORT);

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

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void AisParser::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool AisParser::buildReport() 
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




