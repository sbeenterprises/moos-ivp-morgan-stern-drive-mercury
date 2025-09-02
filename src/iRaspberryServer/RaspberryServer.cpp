/************************************************************/
/*    NAME: Douglas Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RaspberryServer.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "RaspberryServer.h"
#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <mutex>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

using namespace std;

#define SERVER_IP "192.168.143.15"  // Raspberry Pi IP
#define SERVER_PORT 5000

//Variables
int server_socket;
//Initial variables
int desired_rudder = 0; //Rudder Variable
int desired_thrust = 0; //Thrust Variable
string desired_gear = "N"; //Gear variable starts in neutral
string desired_rudder_command;
string desired_thrust_command;
string deploy;
string manual_overide;
string stop_can = "false"; //Variable to stop the boat
bool           has_sent_continue = false;
std::string    last_stop_can      = "true";




mutex command_mutex;

//---------------------------------------------------------
// Constructor()

RaspberryServer::RaspberryServer()
{
    lock_guard<mutex> lock(command_mutex);
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket == -1) {
        cerr << "Failed to create socket" << endl;
        return;
    }

    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

    while (connect(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        cerr << "Failed to connect to Raspberry Pi TCP server, retrying in 5 seconds..." << endl;
        sleep(5);  // Retry after 5 seconds
    }

    cout << "Connected to Raspberry Pi TCP server!" << endl;
}

//---------------------------------------------------------
// Destructor

RaspberryServer::~RaspberryServer()
{
  if (server_socket != -1) {
    close(server_socket);
  }
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool RaspberryServer::OnNewMail(MOOSMSG_LIST &NewMail)
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

     if(key == "DESIRED_RUDDER") 
       desired_rudder = static_cast<int>(round(msg.GetDouble()));

     else if(key == "DESIRED_THRUST")
        desired_thrust = msg.GetDouble();
      else if(key == "DESIRED_GEAR")
        desired_gear = msg.GetString();
     else if(key == "DEPLOY")
        deploy = msg.GetString(); 
     else if(key == "MOOS_MANUAL_OVERIDE")
        manual_overide = msg.GetString(); 
     else if(key == "STOP_CAN")
        stop_can = msg.GetString(); 
     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool RaspberryServer::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool RaspberryServer::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  //make the commands
  //rudder command
  char buffer[12];
  //int mapped_rudder = static_cast<int>((desired_rudder + 30) * 100 / 60); //-30,30 to 0,100
  //i had to invert because it was inverted to control
  //CHANGE HERE IF CHANGES THE UPPER OR LOWER LIMIT OF CONTROL !!
  snprintf(buffer, sizeof(buffer), "ST%03d", desired_rudder); 
  desired_rudder_command = buffer;
  
  //thrust command
  if (desired_gear == "A" || desired_gear == "N" || desired_gear == "R") {
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "P%s%03d", desired_gear.c_str(), static_cast<int>(desired_thrust));
    desired_thrust_command = buffer;
  } else {
    desired_thrust_command = ""; // Default to empty if invalid gear
  }

 
  if (stop_can == "true") {
    // If stop_can is true, we don't send thrust or rudder commands
    desired_thrust_command = "PN000"; // Stop thrust
    desired_rudder_command = "ST000"; // Stop rudder
  } else {
    // If stop_can is false, we send the commands
    sendCommand(desired_rudder_command);
    sendCommand(desired_thrust_command);
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool RaspberryServer::OnStartUp()
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
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void RaspberryServer::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("DESIRED_RUDDER", 0); //Registro da variável do leme
  Register("DESIRED_THRUST", 0); //Registro da variável da máquina
  Register("DESIRED_GEAR", 0); //Registro da leitura de marcha
  Register("DEPLOY", 0); //Registro da variável deploy (start no pmarineviewer)
  Register("MOOS_MANUAL_OVERIDE", 0); //Comando manual da lancha
  Register("STOP_CAN", 0); //Registro do comando de parar a lancha
  // Register("FOOBAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool RaspberryServer::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "desired_thrust_command | desired_rudder_command | stop_can | Delta";
  actab.addHeaderLines();
  actab << desired_thrust_command << desired_rudder_command << stop_can << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}

//------------------------------------------------------------
// Function to send information via TCP socket

void RaspberryServer::sendCommand(std::string msg)
{
    lock_guard<mutex> lock(command_mutex);

    if (server_socket == -1) {
        cerr << "Cannot send command, not connected to server" << endl;
        return;
    }

    ssize_t bytes_sent = send(server_socket, msg.c_str(), msg.size(), 0);

    if (bytes_sent == -1) {
        cerr << "Error sending command. Reconnecting..." << endl;
        close(server_socket);
        server_socket = -1;
        
        // Try to reconnect
        RaspberryServer();  // Call constructor again to reconnect
    }
    
}



