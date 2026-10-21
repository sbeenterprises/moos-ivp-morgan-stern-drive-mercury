/************************************************************/
/*    FILE: NavGuard.h                                      */
/*    pNavGuard: watches the navigation data for faults     */
/*    (NavGuardModel), tells the operator and stops         */
/*    autonomy when it cannot be trusted.                   */
/************************************************************/

#ifndef NavGuard_HEADER
#define NavGuard_HEADER

#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "NavGuardModel.h"

class NavGuard : public AppCastingMOOSApp
{
 public:
  NavGuard();
  ~NavGuard() {}

 protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  void registerVariables();
  bool setSpeedParam(const std::string& param, double val);

 private: // Configuration
  double m_spd_scale;       // NAV_SPEED units -> m/s (knots on the boats)
  std::string m_spd_units;
  bool   m_stop_on_fault;
  std::string m_nmea_var;   // raw NMEA for the RMC status flag; "" = off

 private: // State
  NavGuardModel m_model;
  double m_lat, m_lon;
  bool   m_lat_new, m_lon_new;
  std::string m_allstop;    // IVPHELM_ALLSTOP: "clear" = the helm drives
  NavGuardModel::State m_posted_state;
  double m_last_post, m_last_stop;
  unsigned int m_faults, m_stops;
  std::string m_last_fault;
};

#endif
