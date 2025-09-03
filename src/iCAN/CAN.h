/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: CAN.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef CAN_HEADER
#define CAN_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class CAN : public AppCastingMOOSApp
{
 public:
   CAN();
   ~CAN();

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
