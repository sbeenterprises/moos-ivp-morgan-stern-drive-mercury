/*****************************************************************/
/*    FILE: NavGuardModel.h                                      */
/*                                                               */
/*  Navigation integrity checks. Plain C++ (no MOOS), unit       */
/*  tested in tests/navguard_test.cpp and replayable against     */
/*  logs with navguard_replay.                                   */
/*                                                               */
/*  A single sensor cannot tell that it is wrong, so independent */
/*  pieces of navigation data are compared with each other:      */
/*   heading   NAV_HEADING vs the track the GPS positions make   */
/*             (straight running above min_speed)                */
/*   speed     NAV_SPEED vs the speed the positions make         */
/*   frozen    NAV_SPEED / NAV_COG / NAV_HEADING unchanged while */
/*             the GPS positions show the boat's speed or track  */
/*             changing (a steady value alone is fine)           */
/*   stale     positions stop changing while SOG says moving, or */
/*             no position at all                                */
/*   jump      a step between fixes faster than the boat can go  */
/*   gps_flag  the receiver reports no fix (RMC status V)        */
/*   follow    in autonomy, the track stays far off the course   */
/*             the helm asks for and is not turning toward it    */
/*  A condition first gives WARN, then FAULT once it has lasted  */
/*  its time. FAULT holds until every check has been clean for   */
/*  clear_secs. Speeds are m/s inside the model.                 */
/*****************************************************************/

#ifndef NAV_GUARD_MODEL_HEADER
#define NAV_GUARD_MODEL_HEADER

#include <deque>
#include <string>
#include <vector>

class NavGuardModel
{
 public:
  NavGuardModel();

  // ---- configuration (false = out of range, unchanged)
  bool setMinSpeed(double mps);         // heading / follow checks only above this
  bool setHeadingTol(double deg);
  bool setHeadingSecs(double s);
  bool setSpeedTol(double mps);
  bool setSpeedSecs(double s);
  bool setFreezeSecs(double s);         // speed / course unchanged this long while moving
  bool setHeadingFreezeSecs(double s);
  bool setStaleSecs(double s);
  bool setJumpSpeed(double mps);        // implied speed above this = jump
  bool setJumpMinDist(double m);
  bool setGpsFlagSecs(double s);
  bool setFollowTol(double deg);
  bool setFollowSecs(double s);
  bool setClearSecs(double s);
  bool setWindowSecs(double s);         // positions used for track / speed

  // ---- inputs (now = seconds, any epoch, increasing)
  void position(double now, double lat, double lon);
  void heading(double now, double deg);
  void sog(double now, double mps);
  void cog(double now, double deg);
  void gpsValid(double now, bool valid);
  void desiredHeading(double now, double deg);
  void setAutonomy(bool on) { m_autonomy = on; }

  // ---- evaluation
  enum State { OK = 0, WARN = 1, FAULT = 2 };
  State update(double now);
  State state() const { return m_state; }
  std::string reason() const { return m_reason; }      // worst active check, readable
  std::string stateName() const;

  // the last derived values (for reports); -1 = unknown
  double gpsSpeed() const   { return m_gps_spd; }
  double gpsTrack() const   { return m_gps_trk; }
  double headingError() const { return m_hdg_err; }

  static double angleDiff(double a, double b);   // 0..180

 private:
  struct Fix { double t, x, y; };
  struct Check {
    std::string name;
    double since;     // condition true since (-1 = not active)
    double limit;     // seconds until FAULT
    std::string text;
    Check() : since(-1), limit(0) {}
  };
  void raise(Check& c, double now, bool cond, const std::string& text);
  bool trackFromWindow(double now, double& spd, double& trk, bool& straight) const;

  // config
  double m_min_spd, m_hdg_tol, m_spd_tol, m_jump_spd, m_jump_min, m_follow_tol;
  double m_freeze_secs, m_hdg_freeze_secs, m_stale_secs, m_clear_secs, m_window;

  // inputs
  bool   m_have_origin;
  double m_lat0, m_lon0, m_kx;
  std::deque<Fix> m_fixes;          // changed positions only
  double m_last_pos_mail, m_last_pos_change;
  double m_hdg, m_hdg_time, m_hdg_change, m_hdg_ref_trk;
  double m_sog, m_sog_time, m_sog_change, m_sog_ref_spd;
  double m_cog, m_cog_time, m_cog_change, m_cog_ref_trk;
  bool   m_gps_valid; double m_gps_invalid_since;
  double m_des_hdg, m_des_time, m_des_stable_since, m_des_ref;
  bool   m_autonomy;
  std::deque<std::pair<double,double> > m_follow_hist;   // (time, track error)
  bool   m_jump_pending; double m_jump_dist, m_jump_time;

  // checks
  Check m_c_heading, m_c_speed, m_c_sog_frozen, m_c_cog_frozen, m_c_hdg_frozen,
        m_c_stale, m_c_jump, m_c_gps, m_c_follow;
  std::vector<Check*> m_checks;

  // state
  State m_state;
  std::string m_reason;
  double m_clean_since;
  double m_gps_spd, m_gps_trk, m_hdg_err;
};

#endif
