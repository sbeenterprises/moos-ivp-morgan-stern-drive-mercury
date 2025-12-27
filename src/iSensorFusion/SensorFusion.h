/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SensorFusion.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef SensorFusion_HEADER
#define SensorFusion_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class SensorFusion : public AppCastingMOOSApp
{
 public:
   SensorFusion();
   ~SensorFusion();

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
