/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SensorFusionTracker_ref.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef SensorFusionTracker_ref_HEADER
#define SensorFusionTracker_ref_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class SensorFusionTracker_ref : public AppCastingMOOSApp
{
 public:
   SensorFusionTracker_ref();
   ~SensorFusionTracker_ref();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
};

#endif 
