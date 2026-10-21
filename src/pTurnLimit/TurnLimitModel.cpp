/*****************************************************************/
/*    FILE: TurnLimitModel.cpp                                   */
/*    See TurnLimitModel.h                                       */
/*****************************************************************/

#include "TurnLimitModel.h"
#include <cmath>
#include <cstdlib>
#include <sstream>

using namespace std;

static const double PI = 3.14159265358979;

static string trim(const string& s)
{
  size_t a = s.find_first_not_of(" \t");
  if(a == string::npos)
    return "";
  size_t b = s.find_last_not_of(" \t");
  return s.substr(a, b - a + 1);
}

// heading (deg, 0 = north, clockwise) of the vector dx,dy
static double bearing(double dx, double dy)
{
  double h = atan2(dx, dy) * 180 / PI;
  return (h < 0) ? h + 360 : h;
}

static double angleDiff(double a, double b)
{
  double d = fmod(fabs(a - b), 360.0);
  return (d > 180) ? 360 - d : d;
}

TurnLimitModel::TurnLimitModel()
{
  m_trigger     = 45;
  m_release     = 15;
  m_corner_dist = 100;
  m_corner_min  = 60;
  m_hold        = 3;
  m_des_stale   = 3;
  m_loop_start  = 0;
  m_repeat      = false;
  m_wpt_index   = 0;
  m_des_hdg     = 0;
  m_des_time    = -1e9;
  m_hdg_active  = false;
  m_limited     = false;
  m_last_cause  = -1e9;
  m_hdg_err     = -1;
  m_corner_angle = -1;
  m_dist_wpt    = -1;
}

bool TurnLimitModel::setHeadingTrigger(double deg)
{
  if(deg <= 0 || deg > 180 || deg <= m_release)
    return false;
  m_trigger = deg;
  return true;
}

bool TurnLimitModel::setHeadingRelease(double deg)
{
  if(deg < 0 || deg >= m_trigger)
    return false;
  m_release = deg;
  return true;
}

bool TurnLimitModel::setCornerDist(double m)
{
  if(m < 0) return false;
  m_corner_dist = m;
  return true;
}

bool TurnLimitModel::setCornerAngle(double deg)
{
  if(deg <= 0 || deg > 180) return false;
  m_corner_min = deg;
  return true;
}

bool TurnLimitModel::setHoldSecs(double s)
{
  if(s < 0) return false;
  m_hold = s;
  return true;
}

bool TurnLimitModel::setDesiredStaleSecs(double s)
{
  if(s <= 0) return false;
  m_des_stale = s;
  return true;
}

void TurnLimitModel::setRoute(const vector<double>& xs, const vector<double>& ys)
{
  m_xs = xs;
  m_ys = ys;
  if(m_ys.size() != m_xs.size())
    m_ys.resize(m_xs.size(), 0);
  m_wpt_index = 0;
}

void TurnLimitModel::applyWptUpdate(const string& update)
{
  stringstream ss(update);
  string field;
  while(getline(ss, field, '#')) {
    field = trim(field);
    size_t eq = field.find('=');
    if(eq == string::npos)
      continue;
    string key = trim(field.substr(0, eq));
    string val = trim(field.substr(eq + 1));
    if(key == "polygon" || key == "points" || key == "xpoints") {
      if(val.compare(0, 5, "pts={") == 0)
        val = val.substr(5);
      size_t brace = val.find('}');
      if(brace != string::npos)
        val = val.substr(0, brace);
      vector<double> xs, ys;
      stringstream ps(val);
      string pt;
      bool ok = true;
      while(getline(ps, pt, ':')) {
        size_t c = pt.find(',');
        if(c == string::npos) { ok = false; break; }
        xs.push_back(atof(pt.substr(0, c).c_str()));
        ys.push_back(atof(pt.substr(c + 1).c_str()));
      }
      if(ok && !xs.empty())
        setRoute(xs, ys);
    }
    else if(key == "loop_start")
      setLoopStart(atoi(val.c_str()));
    else if(key == "repeat")
      setRepeat(atoi(val.c_str()) > 0);
  }
}

double TurnLimitModel::turnAt(int ix, double from_x, double from_y) const
{
  int n = (int)m_xs.size();
  if(ix < 0 || ix >= n)
    return -1;
  int next;
  if(ix + 1 < n)
    next = ix + 1;
  else if(m_repeat && n > 1)
    next = (m_loop_start < n) ? m_loop_start : 0;
  else
    return -1;   // end of the route: no turn to make
  if(next == ix)
    return -1;
  double px, py;
  if(ix > 0) { px = m_xs[ix - 1]; py = m_ys[ix - 1]; }
  else       { px = from_x;       py = from_y; }
  double inx = m_xs[ix] - px, iny = m_ys[ix] - py;
  double outx = m_xs[next] - m_xs[ix], outy = m_ys[next] - m_ys[ix];
  if(hypot(inx, iny) < 1e-6 || hypot(outx, outy) < 1e-6)
    return -1;
  return angleDiff(bearing(inx, iny), bearing(outx, outy));
}

bool TurnLimitModel::update(double now, bool autonomy, bool pos_known,
                            double x, double y, bool hdg_known, double heading)
{
  m_hdg_err = -1;
  m_corner_angle = -1;
  m_dist_wpt = -1;
  if(!autonomy) {
    m_hdg_active = false;
    m_limited = false;
    m_last_cause = -1e9;
    m_reason = "";
    return false;
  }

  // heading trigger, with hysteresis
  bool des_fresh = (now - m_des_time) <= m_des_stale;
  if(hdg_known && des_fresh) {
    m_hdg_err = angleDiff(m_des_hdg, heading);
    if(m_hdg_err >= m_trigger)
      m_hdg_active = true;
    else if(m_hdg_err <= m_release)
      m_hdg_active = false;
  }
  else
    m_hdg_active = false;

  // corner trigger
  bool corner = false;
  int n = (int)m_xs.size();
  if(pos_known && m_corner_dist > 0 && m_wpt_index >= 0 && m_wpt_index < n) {
    m_dist_wpt = hypot(m_xs[m_wpt_index] - x, m_ys[m_wpt_index] - y);
    m_corner_angle = turnAt(m_wpt_index, x, y);
    corner = (m_dist_wpt <= m_corner_dist) && (m_corner_angle >= m_corner_min);
  }

  if(corner || m_hdg_active) {
    m_last_cause = now;
    m_reason = corner ? "corner" : "heading";
    m_limited = true;
  }
  else if(m_limited && (now - m_last_cause) < m_hold)
    m_reason = "hold";
  else {
    m_limited = false;
    m_reason = "";
  }
  return m_limited;
}
