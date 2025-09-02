/************************************************************/
/*    NAME: D Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: XYpublisher.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "XYpublisher.h"

using namespace std;

std::string node_report_local;
//---------------------------------------------------------
// Constructor()

XYpublisher::XYpublisher()
{
}

//---------------------------------------------------------
// Destructor

XYpublisher::~XYpublisher()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool XYpublisher::OnNewMail(MOOSMSG_LIST &NewMail)
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

     if(key == "NODE_REPORT_LOCAL") {
        node_report_local = msg.GetString();
        //cout << "NODE_REPORT_LOCAL: " << node_report_local << endl;
        //cout << "NODE_REPORT_LOCAL: " << msg.GetString() << endl;
        //cout << "NODE_REPORT_LOCAL: " << msg.GetDouble() << endl;
      }
      else if(key == "FOO") {
        // Handle FOO message
      }
      else if(key == "BAR") {
        // Handle BAR message
      }
       

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool XYpublisher::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool XYpublisher::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  
  //Get the node_report_local in the string like
  //NAME=alpha,X=-6045.75,Y=17.67,SPD=0.07,HDG=216.34,LAT=29.72072828,LON=-91.20952668,TYPE=ship,COLOR=red,MODE=PARK,ALLSTOP=unknown,INDEX=1014,TIME=1746120319.79,LENGTH=100

  //Get the X and Y values from the string
  std::stringstream ss(node_report_local);
  std::string token;

  std::string x, y;

  //Read the second token (X)
  std::getline(ss, token, ',');
  std::getline(ss, token, ',');

  x = token.substr(token.find('=') + 1);

  //Read the third token (Y)
  //std::getline(ss, token, ',');
  std::getline(ss, token, ',');

  y = token.substr(token.find('=') + 1);

  //Convert the X and Y values to double

  m_nav_x = std::stod(x);

  m_nav_y = std::stod(y);

  //Notify the values
  Notify("NAV_X", m_nav_x);
  Notify("NAV_Y", m_nav_y);

  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool XYpublisher::OnStartUp()
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

void XYpublisher::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT_LOCAL", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool XYpublisher::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "nav_x | nav_y | node_report_local | Delta";
  actab.addHeaderLines();
  actab << m_nav_x << m_nav_y << node_report_local << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}




