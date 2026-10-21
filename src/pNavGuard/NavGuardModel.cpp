/*****************************************************************/
/*    FILE: NavGuardModel.cpp                                    */
/*****************************************************************/

#include <cmath>
#include <algorithm>
#include <cstdio>
#include "NavGuardModel.h"

using namespace std;

static const double kDegToRad = M_PI / 180.0;
static const double kEarthR   = 6378137.0;

// Positions closer than this count as the same fix (receiver repeats).
static const double kSamePos  = 0.05;
// Track over the two halves of the window must agree this well for a
// "straight" run (heading vs track is meaningless in a turn).
static const double kStraightTol = 15;
// DESIRED_HEADING moving by more than this restarts the follow check.
static const double kDesiredMove = 10;
// ... and it must then have been steady this long.
static const double kDesiredSettle = 8;
// A held heading / course is only "frozen" once the GPS track has
// turned this much since it last changed.
static const double kFrozenTurn = 20;
// A check must hold this long before it shows as a warning (turns and
// accelerations briefly upset the comparisons).
static const double kWarnSecs = 2;
// Following: an error that shrank by kFollowProgress over the last
// kFollowLook seconds is a turn in progress, not a failure to follow.
static const double kFollowLook = 5;
static const double kFollowProgress = 5;

NavGuardModel::NavGuardModel()
{
  m_min_spd    = 1.5;    // ~3 kn
  m_hdg_tol    = 45;
  m_spd_tol    = 1.0;    // ~2 kn
  m_jump_spd   = 30;     // ~60 kn
  m_jump_min   = 15;
  m_follow_tol = 60;
  m_freeze_secs = 5;
  m_hdg_freeze_secs = 10;
  m_stale_secs = 3;
  m_clear_secs = 10;
  m_window     = 4;

  m_have_origin = false;
  m_lat0 = m_lon0 = 0; m_kx = 1;
  m_last_pos_mail = m_last_pos_change = -1;
  m_hdg = m_hdg_time = m_hdg_change = m_hdg_ref_trk = -1;
  m_sog = m_sog_time = m_sog_change = m_sog_ref_spd = -1;
  m_cog = m_cog_time = m_cog_change = m_cog_ref_trk = -1;
  m_gps_valid = true; m_gps_invalid_since = -1;
  m_des_hdg = m_des_time = m_des_stable_since = m_des_ref = -1;
  m_autonomy = false;
  m_jump_pending = false; m_jump_dist = 0; m_jump_time = -1;

  m_c_heading.name    = "heading";     m_c_heading.limit = 10;
  m_c_speed.name      = "speed";       m_c_speed.limit = 8;
  m_c_sog_frozen.name = "sog_frozen";  m_c_sog_frozen.limit = 0;
  m_c_cog_frozen.name = "cog_frozen";  m_c_cog_frozen.limit = 0;
  m_c_hdg_frozen.name = "hdg_frozen";  m_c_hdg_frozen.limit = 0;
  m_c_stale.name      = "stale";       m_c_stale.limit = 0;
  m_c_jump.name       = "jump";        m_c_jump.limit = 0;
  m_c_gps.name        = "gps_flag";    m_c_gps.limit = 2;
  m_c_follow.name     = "follow";      m_c_follow.limit = 20;
  // Order = priority of the reason shown when several are active.
  m_checks.push_back(&m_c_jump);
  m_checks.push_back(&m_c_gps);
  m_checks.push_back(&m_c_stale);
  m_checks.push_back(&m_c_heading);
  m_checks.push_back(&m_c_hdg_frozen);
  m_checks.push_back(&m_c_speed);
  m_checks.push_back(&m_c_sog_frozen);
  m_checks.push_back(&m_c_cog_frozen);
  m_checks.push_back(&m_c_follow);

  m_state = OK;
  m_clean_since = -1;
  m_gps_spd = m_gps_trk = m_hdg_err = -1;
}

// ---------------------------------------------------------------- config

static bool inRange(double v, double lo, double hi) { return v >= lo && v <= hi; }

bool NavGuardModel::setMinSpeed(double v)    { if(!inRange(v, 0.2, 20))  return false; m_min_spd = v; return true; }
bool NavGuardModel::setHeadingTol(double v)  { if(!inRange(v, 10, 170))  return false; m_hdg_tol = v; return true; }
bool NavGuardModel::setHeadingSecs(double v) { if(!inRange(v, 1, 120))   return false; m_c_heading.limit = v; return true; }
bool NavGuardModel::setSpeedTol(double v)    { if(!inRange(v, 0.2, 20))  return false; m_spd_tol = v; return true; }
bool NavGuardModel::setSpeedSecs(double v)   { if(!inRange(v, 1, 120))   return false; m_c_speed.limit = v; return true; }
bool NavGuardModel::setFreezeSecs(double v)  { if(!inRange(v, 1, 120))   return false; m_freeze_secs = v; return true; }
bool NavGuardModel::setHeadingFreezeSecs(double v) { if(!inRange(v, 1, 120)) return false; m_hdg_freeze_secs = v; return true; }
bool NavGuardModel::setStaleSecs(double v)   { if(!inRange(v, 0.5, 60))  return false; m_stale_secs = v; return true; }
bool NavGuardModel::setJumpSpeed(double v)   { if(!inRange(v, 2, 200))   return false; m_jump_spd = v; return true; }
bool NavGuardModel::setJumpMinDist(double v) { if(!inRange(v, 1, 1000))  return false; m_jump_min = v; return true; }
bool NavGuardModel::setGpsFlagSecs(double v) { if(!inRange(v, 0, 60))    return false; m_c_gps.limit = v; return true; }
bool NavGuardModel::setFollowTol(double v)   { if(!inRange(v, 20, 170))  return false; m_follow_tol = v; return true; }
bool NavGuardModel::setFollowSecs(double v)  { if(!inRange(v, 5, 300))   return false; m_c_follow.limit = v; return true; }
bool NavGuardModel::setClearSecs(double v)   { if(!inRange(v, 1, 300))   return false; m_clear_secs = v; return true; }
bool NavGuardModel::setWindowSecs(double v)  { if(!inRange(v, 1, 30))    return false; m_window = v; return true; }

// ---------------------------------------------------------------- inputs

double NavGuardModel::angleDiff(double a, double b)
{
  double d = fmod(fabs(a - b), 360.0);
  return d > 180 ? 360 - d : d;
}

void NavGuardModel::position(double now, double lat, double lon)
{
  if(!m_have_origin) {
    m_lat0 = lat; m_lon0 = lon;
    m_kx = cos(lat * kDegToRad);
    m_have_origin = true;
  }
  m_last_pos_mail = now;
  Fix f;
  f.t = now;
  f.x = (lon - m_lon0) * kDegToRad * kEarthR * m_kx;
  f.y = (lat - m_lat0) * kDegToRad * kEarthR;

  if(!m_fixes.empty()) {
    const Fix& p = m_fixes.back();
    double d = hypot(f.x - p.x, f.y - p.y);
    if(d < kSamePos)
      return;                                   // a repeat, not a new fix
    double dt = now - p.t;
    if(d >= m_jump_min && (dt <= 0 || d / dt > m_jump_spd)) {
      m_jump_pending = true;
      m_jump_dist = d;
      m_jump_time = now;
      m_fixes.clear();                          // restart the track after it
    }
  }
  m_last_pos_change = now;
  m_fixes.push_back(f);
  while(m_fixes.size() > 2 && m_fixes.front().t < now - 3 * m_window)
    m_fixes.pop_front();
}

void NavGuardModel::heading(double now, double deg)
{
  if(m_hdg_change < 0 || deg != m_hdg) { m_hdg_change = now; m_hdg_ref_trk = -1; }
  m_hdg = deg; m_hdg_time = now;
}

void NavGuardModel::sog(double now, double mps)
{
  if(m_sog_change < 0 || mps != m_sog) { m_sog_change = now; m_sog_ref_spd = -1; }
  m_sog = mps; m_sog_time = now;
}

void NavGuardModel::cog(double now, double deg)
{
  if(m_cog_change < 0 || deg != m_cog) { m_cog_change = now; m_cog_ref_trk = -1; }
  m_cog = deg; m_cog_time = now;
}

void NavGuardModel::gpsValid(double now, bool valid)
{
  if(!valid && m_gps_valid) m_gps_invalid_since = now;
  if(valid) m_gps_invalid_since = -1;
  m_gps_valid = valid;
}

void NavGuardModel::desiredHeading(double now, double deg)
{
  if(m_des_ref < 0 || angleDiff(deg, m_des_ref) > kDesiredMove) {
    m_des_ref = deg;
    m_des_stable_since = now;
  }
  m_des_hdg = deg; m_des_time = now;
}

// ---------------------------------------------------------------- update

// Speed and track over the last m_window seconds of fixes; straight =
// the two halves of that window point the same way.
bool NavGuardModel::trackFromWindow(double now, double& spd, double& trk,
                                    bool& straight) const
{
  if(m_fixes.size() < 3)
    return false;
  const Fix& last = m_fixes.back();
  if(now - last.t > m_window)
    return false;                               // no recent movement data
  double t0 = last.t - m_window, tm = last.t - m_window / 2;
  const Fix* a = 0; const Fix* m = 0;
  for(size_t i = 0; i < m_fixes.size(); i++) {
    if(!a && m_fixes[i].t >= t0) a = &m_fixes[i];
    if(!m && m_fixes[i].t >= tm) m = &m_fixes[i];
  }
  if(!a || !m || a == &last || last.t - a->t < m_window * 0.6)
    return false;
  double dx = last.x - a->x, dy = last.y - a->y;
  spd = hypot(dx, dy) / (last.t - a->t);
  trk = fmod(atan2(dx, dy) / kDegToRad + 360.0, 360.0);
  straight = false;
  if(m != a && m != &last) {
    double t1 = fmod(atan2(m->x - a->x, m->y - a->y) / kDegToRad + 360.0, 360.0);
    double t2 = fmod(atan2(last.x - m->x, last.y - m->y) / kDegToRad + 360.0, 360.0);
    straight = angleDiff(t1, t2) <= kStraightTol;
  }
  return true;
}

void NavGuardModel::raise(Check& c, double now, bool cond, const string& text)
{
  if(cond) {
    if(c.since < 0) c.since = now;
    c.text = text;
  }
  else {
    c.since = -1;
    c.text.clear();
  }
}

static string fmt(const char* f, double a, double b = 0, double c = 0)
{
  char buf[160];
  snprintf(buf, sizeof(buf), f, a, b, c);
  return buf;
}

NavGuardModel::State NavGuardModel::update(double now)
{
  // ---- derived GPS motion
  double spd = 0, trk = 0; bool straight = false;
  bool have_trk = trackFromWindow(now, spd, trk, straight);
  m_gps_spd = have_trk ? spd : -1;
  m_gps_trk = have_trk ? trk : -1;
  bool moving = have_trk && spd >= m_min_spd;

  // ---- position freshness / jump / receiver flag
  double since_mail = (m_last_pos_mail < 0) ? -1 : now - m_last_pos_mail;
  bool sog_says_moving = m_sog >= 0 && m_sog >= m_min_spd && (now - m_sog_time) < m_stale_secs;
  double since_change = (m_last_pos_change < 0) ? -1 : now - m_last_pos_change;
  if(m_last_pos_mail < 0)
    raise(m_c_stale, now, true, "no GPS position");
  else if(since_mail > m_stale_secs)
    raise(m_c_stale, now, true, fmt("no GPS position for %.0f s", since_mail));
  else if(sog_says_moving && since_change > m_stale_secs)
    raise(m_c_stale, now, true,
          fmt("GPS position not changing for %.0f s while SOG is %.1f m/s", since_change, m_sog));
  else
    raise(m_c_stale, now, false, "");

  if(m_jump_pending) {
    raise(m_c_jump, now, true, fmt("GPS position jumped %.0f m", m_jump_dist));
    m_jump_pending = false;
  }
  else if(m_c_jump.since >= 0 && now - m_jump_time > 1)
    raise(m_c_jump, now, false, "");

  raise(m_c_gps, now, !m_gps_valid, "GPS receiver reports no fix");

  // ---- heading vs GPS track
  m_hdg_err = -1;
  bool hdg_fresh = m_hdg_time >= 0 && now - m_hdg_time < m_stale_secs;
  if(moving && straight && hdg_fresh) {
    m_hdg_err = angleDiff(m_hdg, trk);
    raise(m_c_heading, now, m_hdg_err > m_hdg_tol,
          fmt("heading %.0f deg off GPS track (heading %.0f, track %.0f)", m_hdg_err, m_hdg, trk));
  }
  else if(!moving || !hdg_fresh)
    raise(m_c_heading, now, false, "");
  // in a turn: keep the timer as it is (neither confirm nor clear)

  // ---- reported speed vs GPS speed
  bool sog_fresh = m_sog_time >= 0 && now - m_sog_time < m_stale_secs;
  if(have_trk && sog_fresh) {
    double d = fabs(m_sog - spd);
    raise(m_c_speed, now, d > m_spd_tol,
          fmt("speed %.1f m/s but GPS positions give %.1f m/s", m_sog, spd));
  }
  else
    raise(m_c_speed, now, false, "");

  // ---- frozen values: unchanged while the GPS positions show what they
  //      measure changing. The reference is the GPS speed / track half a
  //      window after the value last changed, so the window no longer
  //      holds mostly older motion.
  double sog_still = m_sog_change < 0 ? 0 : now - m_sog_change;
  double cog_still = m_cog_change < 0 ? 0 : now - m_cog_change;
  double hdg_still = m_hdg_change < 0 ? 0 : now - m_hdg_change;
  if(have_trk) {
    double settle = m_window / 2;
    if(m_sog_change >= 0 && m_sog_ref_spd < 0 && sog_still >= settle) m_sog_ref_spd = spd;
    if(m_cog_change >= 0 && m_cog_ref_trk < 0 && cog_still >= settle) m_cog_ref_trk = trk;
    if(m_hdg_change >= 0 && m_hdg_ref_trk < 0 && hdg_still >= settle) m_hdg_ref_trk = trk;
  }
  bool sog_stuck = have_trk && m_sog_ref_spd >= 0 && sog_still > m_freeze_secs &&
                   fabs(spd - m_sog_ref_spd) > m_spd_tol;
  bool cog_stuck = moving && m_cog_ref_trk >= 0 && cog_still > m_freeze_secs &&
                   angleDiff(trk, m_cog_ref_trk) > kFrozenTurn;
  bool hdg_stuck = moving && m_hdg_ref_trk >= 0 && hdg_still > m_hdg_freeze_secs &&
                   angleDiff(trk, m_hdg_ref_trk) > kFrozenTurn;
  raise(m_c_sog_frozen, now, sog_stuck,
        fmt("speed stuck at %.1f m/s for %.0f s (GPS positions give %.1f m/s)", m_sog, sog_still, spd));
  raise(m_c_cog_frozen, now, cog_stuck,
        fmt("course stuck at %.0f deg for %.0f s (GPS track %.0f)", m_cog, cog_still, trk));
  raise(m_c_hdg_frozen, now, hdg_stuck,
        fmt("heading stuck at %.0f deg for %.0f s while the boat turned (GPS track %.0f)", m_hdg, hdg_still, trk));

  // ---- following: in autonomy, the track keeps diverging from the
  //      course the helm asks for
  bool des_ok = m_autonomy && m_des_time >= 0 && now - m_des_time < 5 &&
                now - m_des_stable_since >= kDesiredSettle;
  if(des_ok && moving) {
    double e = angleDiff(trk, m_des_hdg);
    m_follow_hist.push_back(make_pair(now, e));
    while(m_follow_hist.size() > 1 && m_follow_hist.front().first < now - kFollowLook)
      m_follow_hist.pop_front();
    bool turning_in = m_follow_hist.front().first <= now - kFollowLook + 0.5 &&
                      m_follow_hist.front().second - e >= kFollowProgress;
    if(e > m_follow_tol && !turning_in)
      raise(m_c_follow, now, true,
            fmt("boat not following the commanded course (track %.0f deg off)", e));
    else
      raise(m_c_follow, now, false, "");
  }
  else {
    m_follow_hist.clear();
    raise(m_c_follow, now, false, "");
  }

  // ---- combine
  Check* worst_fault = 0; Check* worst_warn = 0;
  for(size_t i = 0; i < m_checks.size(); i++) {
    Check* c = m_checks[i];
    if(c->since < 0) continue;
    double on = now - c->since;
    if(on >= c->limit) { if(!worst_fault) worst_fault = c; }
    else if(on >= min(kWarnSecs, c->limit / 2) && !worst_warn) worst_warn = c;
  }

  if(worst_fault) {
    m_state = FAULT;
    m_reason = worst_fault->text;
    m_clean_since = -1;
  }
  else if(m_state == FAULT) {
    // a fault holds until everything has been clean for clear_secs
    if(worst_warn) m_clean_since = -1;
    else if(m_clean_since < 0) m_clean_since = now;
    if(m_clean_since >= 0 && now - m_clean_since >= m_clear_secs) {
      m_state = OK;
      m_reason.clear();
    }
    else if(worst_warn)
      m_reason = worst_warn->text;
  }
  else if(worst_warn) {
    m_state = WARN;
    m_reason = worst_warn->text;
  }
  else {
    m_state = OK;
    m_reason.clear();
  }
  return m_state;
}

string NavGuardModel::stateName() const
{
  if(m_state == FAULT) return "fault";
  if(m_state == WARN)  return "warn";
  return "ok";
}
