/************************************************************/
/*    NAME: D Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: SignalK.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef SignalK_HEADER
#define SignalK_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "../../../moos-ivp/MOOS/MOOSGeodesy/libMOOSGeodesy/include/MOOS/libMOOSGeodesy/MOOSGeodesy.h"

class SignalK : public AppCastingMOOSApp
{
 public:
   SignalK();
   ~SignalK();

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
  CMOOSGeodesy m_geodesy;
};

#endif 
