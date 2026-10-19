/************************************************************/
/*    FILE: CorridorModel.h                                 */
/*    DATE: 2026                                            */
/*                                                          */
/*  Route corridor geometry and zone state machine.         */
/*                                                          */
/*  Pure C++11, no MOOS dependency, so it can be unit       */
/*  tested with plain g++ (tests/corridor_test.cpp).        */
/*                                                          */
/*  The corridor is a centreline polyline in local x,y      */
/*  (metres) plus a full width. Each update takes the boat  */
/*  position, computes the cross-track error (minimum       */
/*  distance to any centreline segment) and classifies it:  */
/*                                                          */
/*    in          xte <= slow_frac * half                   */
/*    slow        slow_frac * half < xte <= half            */
/*    recovering  half < xte <= stop_factor * half          */
/*    stop        xte > stop_factor * half                  */
/*                                                          */
/*  with half = width / 2. Moving to a worse zone is        */
/*  immediate; moving back to a better zone needs the xte   */
/*  to clear the boundary by `hysteresis` metres, so the    */
/*  status does not chatter on a boundary.                  */
/************************************************************/

#ifndef CORRIDOR_MODEL_HEADER
#define CORRIDOR_MODEL_HEADER

#include <string>
#include <vector>

class CorridorModel
{
 public:
  enum Zone { ZONE_NONE = 0, ZONE_IN, ZONE_SLOW, ZONE_RECOVERING, ZONE_STOP };

  CorridorModel();

  // Tuning. Each setter returns false (and keeps the old value) if the
  // value is out of range.
  bool setSlowFrac(double v);     // (0, 1]
  bool setStopFactor(double v);   // >= 1
  bool setHysteresis(double v);   // >= 0, metres
  bool setRecoverLead(double v);  // >= 0, metres

  double slowFrac() const    { return m_slow_frac; }
  double stopFactor() const  { return m_stop_factor; }
  double hysteresis() const  { return m_hysteresis; }
  double recoverLead() const { return m_recover_lead; }

  // Geometry. Needs width > 0, at least 2 points and a non-zero length.
  // On failure the previous geometry is cleared, `err` says why, and the
  // model reports ZONE_NONE until a valid geometry is set.
  bool setGeometry(double width, const std::vector<double>& xs,
                   const std::vector<double>& ys, std::string& err);

  // Parses "width=<w>,pts={x1,y1:x2,y2:...}" (the CORRIDOR_UPDATE format
  // posted by iWebSocket) and calls setGeometry().
  bool setGeometryFromSpec(const std::string& spec, std::string& err);

  void clearGeometry();
  bool   geometryValid() const { return m_valid; }
  double width() const         { return m_width; }
  unsigned int size() const    { return (unsigned int)m_xs.size(); }

  // Run one step. When `active` is false (or the geometry is invalid) the
  // zone goes to ZONE_NONE and the hysteresis state is forgotten, so the
  // next activation classifies from scratch.
  Zone update(double x, double y, bool active);

  Zone   zone() const { return m_zone; }
  double xte() const  { return m_xte; }   // last computed, metres, >= 0

  // Point on the centreline `recover_lead` metres further along the track
  // than the closest point to the last position given to update(),
  // clamped to the end of the polyline. False if nothing was computed yet.
  bool recoverPoint(double& rx, double& ry) const;

  // Zone thresholds for the current geometry (metres).
  double slowLimit() const { return m_slow_frac * m_width / 2.0; }
  double halfWidth() const { return m_width / 2.0; }
  double stopLimit() const { return m_stop_factor * m_width / 2.0; }

  // Classification of an xte with no hysteresis.
  Zone rawZone(double xte) const;

  static std::string zoneName(Zone z);

  // Closest point on the polyline to (x,y): segment index, parameter along
  // that segment [0,1], the point itself and the distance. False if the
  // geometry is invalid.
  bool closest(double x, double y, unsigned int& seg, double& t,
               double& px, double& py, double& dist) const;

  // Walk `lead` metres along the polyline from (seg, t), clamped to the end.
  void advance(unsigned int seg, double t, double lead,
               double& rx, double& ry) const;

 private:
  double m_slow_frac;
  double m_stop_factor;
  double m_hysteresis;
  double m_recover_lead;

  bool   m_valid;
  double m_width;
  std::vector<double> m_xs;
  std::vector<double> m_ys;

  Zone   m_zone;
  double m_xte;
  bool   m_have_closest;
  unsigned int m_seg;
  double m_t;
};

#endif
