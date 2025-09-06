/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: CAN.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef CAN_HEADER
#define CAN_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>

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
   bool initCAN();
   void sendRudderCommand(double rudder_value);
   std::string getRudderCANFrame(double rudder_value);
   void sendThrustCommand(double thrust_value);
   std::string getThrustCANFrame(double thrust_value);
   std::string getGearCommand(double thrust_value);

 private: // Configuration variables

 private: // State variables
  double m_desired_rudder;
  double m_desired_thrust;
  int m_can_socket;
  struct sockaddr_can m_can_addr;
  struct ifreq m_ifr;
};

#endif 
