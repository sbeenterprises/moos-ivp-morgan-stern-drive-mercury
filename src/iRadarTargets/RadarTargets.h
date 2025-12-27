/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: RadarTargets.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef RadarTargets_HEADER
#define RadarTargets_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <map>
#include <string>

class RadarTargets : public AppCastingMOOSApp
{
 public:
   RadarTargets();
   ~RadarTargets();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   bool setupUDPSocket();
   bool parseRATTM(const std::string& sentence);

 private: // Configuration variables
   int m_udp_port;
   std::string m_udp_host;

 private: // State variables
   int m_socket_fd;
   double m_own_lat;
   double m_own_lon;
   double m_own_x;
   double m_own_y;
   std::map<std::string, double> m_last_report_time;

   private:
    // Radar target storage
    struct RadarTarget {
        double x, y;           // Local position
        double vx, vy;         // Velocity
        double range, bearing; // Polar coordinates
        double last_update;    // Timestamp
    };
    std::map<std::string, RadarTarget> m_radar_targets;

    void publishRadarTargets();
};

#endif 
