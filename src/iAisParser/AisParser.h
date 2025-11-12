/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: AisParser.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef AisParser_HEADER
#define AisParser_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

class AisParser : public AppCastingMOOSApp
{
 public:
   AisParser();
   ~AisParser();

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
