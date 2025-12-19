/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: VisionDetector_ref.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef VisionDetector_ref_HEADER
#define VisionDetector_ref_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class VisionDetector_ref : public AppCastingMOOSApp
{
 public:
   VisionDetector_ref();
   ~VisionDetector_ref();

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
