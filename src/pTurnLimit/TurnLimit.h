/************************************************************/
/*    FILE: TurnLimit.h                                     */
/*    pTurnLimit: holds the boat to turn_speed at sharp     */
/*    route corners and during hard turns (TurnLimitModel). */
/************************************************************/

#ifndef TurnLimit_HEADER
#define TurnLimit_HEADER

#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "TurnLimitModel.h"

class TurnLimit : public AppCastingMOOSApp
{
 public:
  TurnLimit();
  ~TurnLimit() {}

 protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  void registerVariables();
  void postSpeed();

 private: // Configuration
  double m_turn_speed;      // helm speed units; <= 0 = not configured, never limits

 private: // State
  TurnLimitModel m_model;
  bool   m_waypoint, m_trailing, m_stationing, m_manual;
  bool   m_x_known, m_y_known, m_hdg_known;
  double m_nav_x, m_nav_y, m_nav_hdg;
  bool   m_posted;          // last TURN_LIMIT posted
  bool   m_posted_once;
  std::string m_posted_reason;
  unsigned int m_limit_events;
};

#endif
