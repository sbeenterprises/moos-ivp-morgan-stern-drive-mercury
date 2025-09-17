/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: CANRec.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef CANRec_HEADER
#define CANRec_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <map>
#include <string>

class CANRec : public AppCastingMOOSApp
{
 public:
   CANRec();
   ~CANRec();

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
   void readCANMessages();
   bool extractPGN(const can_frame& frame, uint32_t& pgn_out) const;
   void parseEEC2(const can_frame& frame);
   void parseEEC1(const can_frame& frame);
   void parseDM1(const can_frame& frame);
   void parseET1(const can_frame& frame);
   void parseEFLP1(const can_frame& frame);
   void parseLFE(const can_frame& frame);
   void parseIC1(const can_frame& frame);
   void parseVEP(const can_frame& frame);

 private: // Configuration variables
  std::string m_can_interface;

 private: // State variables
  int m_can_socket;
   struct sockaddr_can m_can_addr;
   struct ifreq m_ifr;

   // Engine data
   double m_accelerator_pedal_position;  // SPN 91
   double m_percent_load;                // SPN 92
   double m_engine_speed;                // SPN 190
   double m_engine_percent_torque;       // SPN 513
   double m_engine_coolant_temp;         // SPN 110
   double m_oil_temp;                    // SPN 175
   double m_oil_pressure;                // SPN 100
   double m_fuel_rate;                   // SPN 183
   double m_intake_manifold_pressure;    // SPN 102
   double m_battery_potential;           // SPN 168

   // DTC tracking
   std::map<uint32_t, std::string> m_active_dtcs;

   // Message counters
   unsigned int m_eec2_count;
   unsigned int m_eec1_count;
   unsigned int m_dm1_count;
   unsigned int m_et1_count;
   unsigned int m_eflp1_count;
   unsigned int m_lfe_count;
   unsigned int m_ic1_count;
   unsigned int m_vep_count;
};

#endif 
