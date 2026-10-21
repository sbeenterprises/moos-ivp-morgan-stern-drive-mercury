/************************************************************/
/*    FILE: ColavScale.h                                    */
/*    pColavScale: per-contact collision-avoidance distances*/
/*    from closing speed (ColavScaleModel).                 */
/************************************************************/

#ifndef ColavScale_HEADER
#define ColavScale_HEADER

#include <map>
#include <set>
#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "ColavScaleModel.h"

class ColavScale : public AppCastingMOOSApp
{
 public:
  ColavScale();
  ~ColavScale() {}

 protected:
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  bool buildReport();

 protected:
  void registerVariables();
  void handleNodeReport(const std::string& report);
  void handleLifeEvent(const std::string& event);
  void handleColavDist(const std::string& echo);
  void postAlertRange();

 private: // Configuration
  ColavScaleModel m_model;
  double m_nav_speed_factor;      // NAV_SPEED -> m/s (knots on the boat)
  double m_contact_speed_factor;  // NODE_REPORT SPD -> m/s
  std::string m_prefix;           // spawned behaviour name prefix (avdcol_)
  double m_cpa_range;             // alert cpa_range (>= completed_dist)
  double m_contact_max_age;
  double m_alert_repost_secs;

 private: // State
  struct Contact { double x, y, hdg, spd, time; };
  std::map<std::string, Contact> m_contacts;
  std::set<std::string> m_live;                   // contacts with an avdcol_ instance
  std::map<std::string, ColavDist> m_posted;      // last distances sent per contact
  bool   m_nav_known[4];                          // x, y, heading, speed
  double m_nav_x, m_nav_y, m_nav_hdg, m_nav_spd;
  double m_last_alert_post;
  unsigned int m_updates_posted;
  std::string m_summary;
};

#endif
