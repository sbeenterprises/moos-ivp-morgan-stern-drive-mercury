/*****************************************************************/
/*    FILE: TurnLimitModel.h                                     */
/*                                                               */
/*  Decides when the boat should be held to a turn speed. Plain  */
/*  C++ (no MOOS), unit tested in tests/turnlimit_test.cpp.      */
/*                                                               */
/*  Two triggers, either one limits the speed:                   */
/*   corner : within corner_dist of the route waypoint the boat  */
/*            is heading for, where the route turns by           */
/*            corner_angle or more (works with or without a      */
/*            corridor; covers the loop closing of a repeat)     */
/*   heading: the helm's desired heading is heading_trigger or   */
/*            more away from the boat's heading (any hard turn:  */
/*            region turn-away, collision avoidance, follow,     */
/*            recovery); released below heading_release          */
/*  The limit is held for hold_secs after both clear, and only   */
/*  applies while an autonomous mode drives the boat.            */
/*****************************************************************/

#ifndef TURN_LIMIT_MODEL_HEADER
#define TURN_LIMIT_MODEL_HEADER

#include <string>
#include <vector>

class TurnLimitModel
{
 public:
  TurnLimitModel();

  // Configuration; false (and unchanged) for an out-of-range value.
  bool setHeadingTrigger(double deg);   // (0,180]
  bool setHeadingRelease(double deg);   // [0, trigger)
  bool setCornerDist(double m);         // >= 0, 0 = corners off
  bool setCornerAngle(double deg);      // (0,180]
  bool setHoldSecs(double s);           // >= 0
  bool setDesiredStaleSecs(double s);   // > 0

  // Route (local x,y) as the helm has it; a new route resets the index.
  void setRoute(const std::vector<double>& xs, const std::vector<double>& ys);
  void setLoopStart(int ix)   { m_loop_start = (ix < 0) ? 0 : ix; }
  void setRepeat(bool repeat) { m_repeat = repeat; }
  void setWptIndex(int ix)    { m_wpt_index = ix; }

  // Parses a WPT_UPDATE value ("polygon=x,y:x,y # repeat=1 # ..."),
  // taking the route (polygon= / points= / xpoints=), loop_start= and
  // repeat=. Other fields (speed, capture_radius...) are ignored.
  void applyWptUpdate(const std::string& update);

  void setDesiredHeading(double deg, double now) { m_des_hdg = deg; m_des_time = now; }

  // One evaluation. autonomy: an autonomous mode is driving the boat.
  // Returns true while the speed should be limited.
  bool update(double now, bool autonomy, bool pos_known, double x, double y,
              bool hdg_known, double heading);

  bool limited() const             { return m_limited; }
  std::string reason() const       { return m_reason; }      // corner|heading|hold|""
  double headingError() const      { return m_hdg_err; }     // last, degrees (-1 unknown)
  double cornerAngle() const       { return m_corner_angle; }// at the target waypoint
  double distToWpt() const         { return m_dist_wpt; }
  unsigned int routeSize() const   { return (unsigned int)m_xs.size(); }

  // Turn of the route at waypoint ix, degrees in [0,180]; -1 if the
  // route does not continue past it (or ix is out of range). Incoming
  // leg: from the previous point, or from (from_x,from_y) at the first
  // point; outgoing leg: to the next point, or to loop_start when the
  // route repeats.
  double turnAt(int ix, double from_x, double from_y) const;

 private:
  double m_trigger, m_release, m_corner_dist, m_corner_min, m_hold, m_des_stale;
  std::vector<double> m_xs, m_ys;
  int    m_loop_start;
  bool   m_repeat;
  int    m_wpt_index;
  double m_des_hdg, m_des_time;
  bool   m_hdg_active;
  bool   m_limited;
  double m_last_cause;   // last time a trigger was active
  std::string m_reason;
  double m_hdg_err, m_corner_angle, m_dist_wpt;
};

#endif
