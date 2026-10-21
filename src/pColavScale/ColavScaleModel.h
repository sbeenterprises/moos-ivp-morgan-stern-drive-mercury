/*****************************************************************/
/*    FILE: ColavScaleModel.h                                    */
/*                                                               */
/*  Collision-avoidance engagement distances from closing speed. */
/*  Plain C++ (no MOOS), unit tested in tests/colavscale_test.cpp*/
/*                                                               */
/*  A fixed pwt_outer_dist / pwt_inner_dist gives less warning   */
/*  the faster two boats close. Here they come from time:        */
/*    outer = clamp(outer_secs * v, outer_min, outer_max)        */
/*    inner = clamp(inner_secs * v, inner_min, min(inner_max,    */
/*                                                 outer))       */
/*  v = speed of the contact relative to us while the range is   */
/*  closing (the full relative speed, not just its range-rate    */
/*  part, so crossing traffic is not underestimated); 0 while    */
/*  the range opens, which gives the minimums. Speeds in m/s.    */
/*****************************************************************/

#ifndef COLAV_SCALE_MODEL_HEADER
#define COLAV_SCALE_MODEL_HEADER

struct ColavDist {
  double outer, inner;
  double rel_speed;   // m/s used (0 when opening)
  ColavDist() : outer(0), inner(0), rel_speed(0) {}
};

class ColavScaleModel
{
 public:
  ColavScaleModel();

  // false (and unchanged) for an out-of-range value
  bool setOuterSecs(double s);
  bool setInnerSecs(double s);
  bool setOuterMin(double m);
  bool setInnerMin(double m);
  bool setOuterMax(double m);
  bool setInnerMax(double m);
  bool setChangeFrac(double f);   // re-post when a distance moves by this share
  bool setChangeMin(double m);    // ... and by at least this many metres

  double outerMin() const { return m_outer_min; }
  double innerMin() const { return m_inner_min; }
  double outerMax() const { return m_outer_max; }
  double innerMax() const { return m_inner_max; }
  double outerSecs() const { return m_outer_secs; }
  double innerSecs() const { return m_inner_secs; }

  // headings in degrees (0 north, clockwise), speeds m/s, positions metres
  ColavDist distances(double os_x, double os_y, double os_hdg, double os_spd,
                      double cn_x, double cn_y, double cn_hdg, double cn_spd) const;

  // Whether `now` differs enough from what was last posted.
  bool changed(const ColavDist& posted, const ColavDist& now) const;

 private:
  double m_outer_secs, m_inner_secs;
  double m_outer_min, m_inner_min, m_outer_max, m_inner_max;
  double m_change_frac, m_change_min;
};

#endif
