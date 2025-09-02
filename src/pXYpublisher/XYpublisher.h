/************************************************************/
/*    NAME: D Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: XYpublisher.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef XYpublisher_HEADER
#define XYpublisher_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class XYpublisher : public AppCastingMOOSApp
{
 public:
   XYpublisher();
   ~XYpublisher();

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
  double m_nav_x {0.0};
  double m_nav_y {0.0};

 private: // State variables
};

#endif 
