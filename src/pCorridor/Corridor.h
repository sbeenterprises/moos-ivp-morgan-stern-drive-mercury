/************************************************************/
/*    FILE: Corridor.h                                      */
/*    DATE: 2026                                            */
/*                                                          */
/*  Route corridor monitor.                                 */
/*                                                          */
/*  The UI draws a corridor (centreline + full width) round */
/*  the route; iWebSocket converts it to local x,y and      */
/*  posts CORRIDOR_UPDATE and CORRIDOR. While a waypoint    */
/*  mission runs inside an active corridor this app reports */
/*  the cross-track error and a zone (CORRIDOR_STATUS) that */
/*  the corridor_* behaviours in alpha.bhv act on:          */
/*  slow down, steer back to the centreline, or stop.       */
/*                                                          */
/*  All geometry and zone logic is in CorridorModel.        */
/************************************************************/

#ifndef Corridor_HEADER
#define Corridor_HEADER

#include <string>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "CorridorModel.h"

class Corridor : public AppCastingMOOSApp
{
 public:
  Corridor();
  ~Corridor() {}

 protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  void registerVariables();
  void handleCorridorUpdate(const std::string& sval);
  static bool parseFlag(CMOOSMsg& msg);

 private: // Configuration
  double m_status_interval;   // seconds between unconditional status posts
  double m_recover_move;      // metres the recover point must move to be re-posted

 private: // State
  CorridorModel m_model;

  double m_nav_x, m_nav_y;
  bool   m_nav_x_known, m_nav_y_known;
  bool   m_corridor_on;
  bool   m_waypoint_on;
  std::string m_last_spec;
  std::string m_last_error;

  std::string m_status_posted;
  double m_last_status_post;
  bool   m_was_active;

  bool   m_recover_posted;
  double m_recover_x, m_recover_y;
  unsigned int m_recover_posts;
};

#endif
