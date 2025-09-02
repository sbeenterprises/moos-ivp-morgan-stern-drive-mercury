/************************************************************/
/*    NAME: Douglas Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RaspberryServer.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef RaspberryServer_HEADER
#define RaspberryServer_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class RaspberryServer : public AppCastingMOOSApp
{
 public:
   RaspberryServer();
   ~RaspberryServer();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   void sendCommand(std::string serial_write);

 private: // Configuration variables

 private: // State variables
};

#endif 
