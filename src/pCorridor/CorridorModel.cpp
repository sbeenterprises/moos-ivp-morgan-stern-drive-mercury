/************************************************************/
/*    FILE: CorridorModel.cpp                               */
/*    DATE: 2026                                            */
/************************************************************/

#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <sstream>
#include "CorridorModel.h"

using namespace std;

namespace {

// Strict number parse: the whole (trimmed) string must be a finite number.
bool parseNumber(const string& raw, double& out)
{
  size_t b = raw.find_first_not_of(" \t");
  size_t e = raw.find_last_not_of(" \t");
  if(b == string::npos)
    return(false);
  string s = raw.substr(b, e - b + 1);
  const char* c = s.c_str();
  char* end = 0;
  errno = 0;
  double v = strtod(c, &end);
  if(end == c || *end != '\0' || errno == ERANGE || !std::isfinite(v))
    return(false);
  out = v;
  return(true);
}

} // namespace

//---------------------------------------------------------
// Constructor

CorridorModel::CorridorModel()
{
  m_slow_frac    = 0.5;
  m_stop_factor  = 2.0;
  m_hysteresis   = 2.0;
  m_recover_lead = 20.0;
  m_turn_dist         = 0;
  m_turn_min_deg      = 180;
  m_turn_allowance    = 0;
  m_allowance_applied = 0;

  m_valid = false;
  m_width = 0;

  m_zone = ZONE_NONE;
  m_xte  = 0;
  m_have_closest = false;
  m_seg = 0;
  m_t   = 0;
}

//---------------------------------------------------------
// Tuning setters

bool CorridorModel::setSlowFrac(double v)
{
  if(!(v > 0) || v > 1)
    return(false);
  m_slow_frac = v;
  return(true);
}

bool CorridorModel::setStopFactor(double v)
{
  if(!(v >= 1))
    return(false);
  m_stop_factor = v;
  return(true);
}

bool CorridorModel::setHysteresis(double v)
{
  if(!(v >= 0))
    return(false);
  m_hysteresis = v;
  return(true);
}

bool CorridorModel::setRecoverLead(double v)
{
  if(!(v >= 0))
    return(false);
  m_recover_lead = v;
  return(true);
}

//---------------------------------------------------------
// Procedure: clearGeometry()

void CorridorModel::clearGeometry()
{
  m_valid = false;
  m_width = 0;
  m_xs.clear();
  m_ys.clear();
  m_zone = ZONE_NONE;
  m_xte  = 0;
  m_have_closest = false;
}

//---------------------------------------------------------
// Procedure: setGeometry()

bool CorridorModel::setGeometry(double width, const vector<double>& xs,
                                const vector<double>& ys, string& err)
{
  clearGeometry();

  if(!std::isfinite(width) || !(width > 0)) {
    err = "width must be > 0";
    return(false);
  }
  if(xs.size() != ys.size()) {
    err = "x and y lists differ in length";
    return(false);
  }
  if(xs.size() < 2) {
    err = "need at least 2 centreline points";
    return(false);
  }

  double length = 0;
  for(unsigned int i=0; i<xs.size(); i++) {
    if(!std::isfinite(xs[i]) || !std::isfinite(ys[i])) {
      err = "non-finite centreline point";
      return(false);
    }
    if(i > 0)
      length += hypot(xs[i]-xs[i-1], ys[i]-ys[i-1]);
  }
  if(length < 0.01) {
    err = "centreline has zero length";
    return(false);
  }

  m_width = width;
  m_xs = xs;
  m_ys = ys;
  m_valid = true;
  err = "";
  return(true);
}

//---------------------------------------------------------
// Procedure: setGeometryFromSpec()
//   "width=40.0,pts={x1,y1:x2,y2:...}"

bool CorridorModel::setGeometryFromSpec(const string& spec, string& err)
{
  size_t wpos = spec.find("width=");
  size_t ppos = spec.find("pts={");
  if(wpos == string::npos || ppos == string::npos) {
    clearGeometry();
    err = "expected width=<w>,pts={x,y:...}";
    return(false);
  }

  size_t wend = spec.find(',', wpos);
  if(wend == string::npos || wend > ppos)
    wend = ppos;
  string wstr = spec.substr(wpos + 6, wend - (wpos + 6));
  double width = 0;
  if(!parseNumber(wstr, width)) {
    clearGeometry();
    err = "bad width: " + wstr;
    return(false);
  }

  size_t open  = ppos + 4;   // the '{'
  size_t close = spec.find('}', open);
  if(close == string::npos) {
    clearGeometry();
    err = "pts block not closed";
    return(false);
  }
  string body = spec.substr(open + 1, close - open - 1);

  vector<double> xs, ys;
  stringstream ss(body);
  string tok;
  while(getline(ss, tok, ':')) {
    if(tok.find_first_not_of(" \t") == string::npos)
      continue;
    size_t comma = tok.find(',');
    double x = 0, y = 0;
    if(comma == string::npos ||
       !parseNumber(tok.substr(0, comma), x) ||
       !parseNumber(tok.substr(comma + 1), y)) {
      clearGeometry();
      err = "bad point: " + tok;
      return(false);
    }
    xs.push_back(x);
    ys.push_back(y);
  }

  return(setGeometry(width, xs, ys, err));
}

//---------------------------------------------------------
// Procedure: closest()

//---------------------------------------------------------
// Procedure: turnAngle()

double CorridorModel::turnAngle(unsigned int i) const
{
  unsigned int n = (unsigned int)m_xs.size();
  if(!m_valid || n < 3 || i >= n)
    return(0);

  bool closed = hypot(m_xs[0] - m_xs[n-1], m_ys[0] - m_ys[n-1]) < 1.0;
  unsigned int ia, ib;   // vertex before and after i
  if(i > 0 && i < n-1) { ia = i-1; ib = i+1; }
  else if(closed)      { ia = n-2; ib = 1; }   // the joint of a closed loop
  else
    return(0);

  double ax = m_xs[i] - m_xs[ia], ay = m_ys[i] - m_ys[ia];
  double bx = m_xs[ib] - m_xs[i], by = m_ys[ib] - m_ys[i];
  double la = hypot(ax, ay), lb = hypot(bx, by);
  if(la <= 0 || lb <= 0)
    return(0);
  double c = (ax*bx + ay*by) / (la*lb);
  if(c > 1)  c = 1;
  if(c < -1) c = -1;
  return(acos(c) * 180.0 / M_PI);
}

//---------------------------------------------------------
// Procedure: nearSharpTurn()

bool CorridorModel::nearSharpTurn(double x, double y, double dist,
                                  double min_deg) const
{
  if(!m_valid || dist <= 0)
    return(false);
  for(unsigned int i = 0; i < m_xs.size(); i++) {
    if(hypot(x - m_xs[i], y - m_ys[i]) > dist)
      continue;
    if(turnAngle(i) >= min_deg)
      return(true);
  }
  return(false);
}

//---------------------------------------------------------
// Procedure: closest()

bool CorridorModel::closest(double x, double y, unsigned int& seg, double& t,
                            double& px, double& py, double& dist) const
{
  if(!m_valid)
    return(false);

  bool found = false;
  for(unsigned int i=0; i+1<m_xs.size(); i++) {
    double ax = m_xs[i],   ay = m_ys[i];
    double bx = m_xs[i+1], by = m_ys[i+1];
    double dx = bx - ax,   dy = by - ay;
    double len2 = dx*dx + dy*dy;

    double ti = 0;
    if(len2 > 0) {
      ti = ((x - ax) * dx + (y - ay) * dy) / len2;
      if(ti < 0) ti = 0;
      if(ti > 1) ti = 1;
    }
    double cx = ax + ti * dx;
    double cy = ay + ti * dy;
    double d  = hypot(x - cx, y - cy);

    // Strictly less: on a tie (a vertex shared by two segments) the earlier
    // segment wins, which is harmless - advance() walks on from there.
    if(!found || d < dist) {
      found = true;
      seg = i; t = ti; px = cx; py = cy; dist = d;
    }
  }
  return(found);
}

//---------------------------------------------------------
// Procedure: advance()

void CorridorModel::advance(unsigned int seg, double t, double lead,
                            double& rx, double& ry) const
{
  if(m_xs.size() < 2) {
    rx = 0; ry = 0;
    return;
  }
  if(seg + 1 >= m_xs.size()) {
    seg = (unsigned int)m_xs.size() - 2;
    t = 1;
  }

  double remaining = (lead > 0) ? lead : 0;
  for(unsigned int i=seg; i+1<m_xs.size(); i++) {
    double ax = m_xs[i],   ay = m_ys[i];
    double bx = m_xs[i+1], by = m_ys[i+1];
    double len = hypot(bx - ax, by - ay);
    double start = (i == seg) ? t * len : 0;
    double left  = len - start;

    if(remaining <= left && len > 0) {
      double f = (start + remaining) / len;
      rx = ax + f * (bx - ax);
      ry = ay + f * (by - ay);
      return;
    }
    remaining -= (left > 0) ? left : 0;
  }

  // Ran off the end: clamp to the last vertex.
  rx = m_xs.back();
  ry = m_ys.back();
}

//---------------------------------------------------------
// Procedure: rawZone()

CorridorModel::Zone CorridorModel::rawZone(double xte) const
{
  if(!m_valid)
    return(ZONE_NONE);
  if(xte <= slowLimit())
    return(ZONE_IN);
  if(xte <= halfWidth())
    return(ZONE_SLOW);
  if(xte <= stopLimit())
    return(ZONE_RECOVERING);
  return(ZONE_STOP);
}

//---------------------------------------------------------
// Procedure: update()

CorridorModel::Zone CorridorModel::update(double x, double y, bool active)
{
  if(!active || !m_valid) {
    m_zone = ZONE_NONE;
    m_have_closest = false;
    return(m_zone);
  }

  double px = 0, py = 0, d = 0;
  unsigned int seg = 0;
  double t = 0;
  if(!closest(x, y, seg, t, px, py, d)) {
    m_zone = ZONE_NONE;
    m_have_closest = false;
    return(m_zone);
  }
  m_xte = d;
  m_seg = seg;
  m_t   = t;
  m_have_closest = true;

  // At a sharp corner the boat needs its turning circle: judge the zone
  // on the xte less the allowance (xte() still reports the true distance).
  m_allowance_applied = 0;
  if(m_turn_allowance > 0 && nearSharpTurn(x, y, m_turn_dist, m_turn_min_deg))
    m_allowance_applied = m_turn_allowance;
  double dz = d - m_allowance_applied;
  if(dz < 0)
    dz = 0;

  Zone raw = rawZone(dz);
  if(m_zone == ZONE_NONE || raw >= m_zone) {
    // Fresh start, or getting worse: take it straight away.
    m_zone = raw;
  }
  else {
    // Getting better: only as far as the xte clears each boundary by the
    // hysteresis margin, and never worse than where we already are.
    Zone relaxed = rawZone(dz + m_hysteresis);
    m_zone = (relaxed < m_zone) ? relaxed : m_zone;
  }
  return(m_zone);
}

//---------------------------------------------------------
// Procedure: setTurnAllowance()

bool CorridorModel::setTurnAllowance(double dist, double min_deg, double allowance)
{
  if(dist < 0 || min_deg <= 0 || min_deg > 180 || allowance < 0)
    return(false);
  m_turn_dist      = dist;
  m_turn_min_deg   = min_deg;
  m_turn_allowance = allowance;
  return(true);
}

//---------------------------------------------------------
// Procedure: recoverPoint()

bool CorridorModel::recoverPoint(double& rx, double& ry) const
{
  if(!m_valid || !m_have_closest)
    return(false);
  advance(m_seg, m_t, m_recover_lead, rx, ry);
  return(true);
}

//---------------------------------------------------------
// Procedure: zoneName()

string CorridorModel::zoneName(Zone z)
{
  switch(z) {
  case ZONE_IN:         return("in");
  case ZONE_SLOW:       return("slow");
  case ZONE_RECOVERING: return("recovering");
  case ZONE_STOP:       return("stop");
  default:              return("none");
  }
}
