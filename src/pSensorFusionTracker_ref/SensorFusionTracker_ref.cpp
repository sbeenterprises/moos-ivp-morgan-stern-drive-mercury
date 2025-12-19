/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SensorFusionTracker_ref.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "SensorFusionTracker_ref.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

SensorFusionTracker_ref::SensorFusionTracker_ref()
{
  next_track_id_ = 1;
    association_threshold_ = 15.0;
    track_timeout_ = 5.0;
    quality_threshold_ = 0.5;
    buffer_time_window_ = 0.5;
    radar_noise_pos_ = 3.0;
    radar_noise_vel_ = 0.5;
    vision_noise_pos_ = 25.0;
    process_noise_pos_ = 0.5;
    process_noise_vel_ = 0.2;
    process_noise_yaw_ = 0.1;
    min_hits_to_confirm_ = 3;
    max_misses_ = 5;
    nav_x_ = 0.0;
    nav_y_ = 0.0;
    nav_heading_ = 0.0;
    radar_count_ = 0;
    vision_count_ = 0;
    fusion_count_ = 0;
    tracks_created_ = 0;
    tracks_deleted_ = 0;
}

//---------------------------------------------------------
// Destructor

SensorFusionTracker_ref::~SensorFusionTracker_ref()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool SensorFusionTracker_ref::OnNewMail(MOOSMSG_LIST &NewMail)
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

bool SensorFusionTracker_ref::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool SensorFusionTracker_ref::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool SensorFusionTracker_ref::OnStartUp()
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

void SensorFusionTracker_ref::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  // Register("FOOBAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool SensorFusionTracker_ref::buildReport() 
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




