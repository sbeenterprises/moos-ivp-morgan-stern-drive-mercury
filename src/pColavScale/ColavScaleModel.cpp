/*****************************************************************/
/*    FILE: ColavScaleModel.cpp                                  */
/*    See ColavScaleModel.h                                      */
/*****************************************************************/

#include "ColavScaleModel.h"
#include <cmath>

static const double PI = 3.14159265358979;

ColavScaleModel::ColavScaleModel()
{
  m_outer_secs  = 60;
  m_inner_secs  = 30;
  m_outer_min   = 150;
  m_inner_min   = 75;
  m_outer_max   = 1400;
  m_inner_max   = 700;
  m_change_frac = 0.1;
  m_change_min  = 20;
}

bool ColavScaleModel::setOuterSecs(double s) { if(s <= 0) return false; m_outer_secs = s; return true; }
bool ColavScaleModel::setInnerSecs(double s) { if(s <= 0) return false; m_inner_secs = s; return true; }
bool ColavScaleModel::setOuterMin(double m)  { if(m <= 0) return false; m_outer_min = m; return true; }
bool ColavScaleModel::setInnerMin(double m)  { if(m <= 0) return false; m_inner_min = m; return true; }
bool ColavScaleModel::setOuterMax(double m)  { if(m <= 0) return false; m_outer_max = m; return true; }
bool ColavScaleModel::setInnerMax(double m)  { if(m <= 0) return false; m_inner_max = m; return true; }
bool ColavScaleModel::setChangeFrac(double f){ if(f < 0) return false; m_change_frac = f; return true; }
bool ColavScaleModel::setChangeMin(double m) { if(m < 0) return false; m_change_min = m; return true; }

ColavDist ColavScaleModel::distances(double os_x, double os_y, double os_hdg, double os_spd,
                                     double cn_x, double cn_y, double cn_hdg, double cn_spd) const
{
  // velocity components: x east, y north
  double ovx = os_spd * sin(os_hdg * PI / 180), ovy = os_spd * cos(os_hdg * PI / 180);
  double cvx = cn_spd * sin(cn_hdg * PI / 180), cvy = cn_spd * cos(cn_hdg * PI / 180);
  double rvx = cvx - ovx, rvy = cvy - ovy;          // contact relative to us
  double rx = cn_x - os_x, ry = cn_y - os_y;        // contact from us
  double closing = -(rx * rvx + ry * rvy);          // > 0: range decreasing

  ColavDist d;
  d.rel_speed = (closing > 0) ? hypot(rvx, rvy) : 0;

  double outer_max = (m_outer_max > m_outer_min) ? m_outer_max : m_outer_min;
  d.outer = m_outer_secs * d.rel_speed;
  if(d.outer < m_outer_min) d.outer = m_outer_min;
  if(d.outer > outer_max)   d.outer = outer_max;

  double inner_max = m_inner_max;
  if(inner_max > d.outer) inner_max = d.outer;
  d.inner = m_inner_secs * d.rel_speed;
  if(d.inner < m_inner_min) d.inner = m_inner_min;
  if(d.inner > inner_max)   d.inner = inner_max;
  return d;
}

bool ColavScaleModel::changed(const ColavDist& posted, const ColavDist& now) const
{
  double dout = fabs(now.outer - posted.outer);
  double din  = fabs(now.inner - posted.inner);
  bool out_big = dout >= m_change_min && dout >= m_change_frac * posted.outer;
  bool in_big  = din  >= m_change_min && din  >= m_change_frac * posted.inner;
  return out_big || in_big;
}
