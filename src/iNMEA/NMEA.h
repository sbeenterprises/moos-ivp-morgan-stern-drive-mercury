/************************************************************/
/*    NAME: DLima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: NMEA.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef NMEA_HEADER
#define NMEA_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <netinet/in.h>   // for sockaddr_in

class NMEA : public AppCastingMOOSApp
{
 public:
   NMEA();
   ~NMEA();

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
  int             m_udp_sock;         // Socket file descriptor
  struct sockaddr_in m_server_addr;   // Server address
  bool            m_socket_initialized; 

 private: // State variables
};

#endif 
