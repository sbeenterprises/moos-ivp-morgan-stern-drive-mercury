/************************************************************/
/*    FILE: DriveCore.cpp                                   */
/*    DATE: October 2026                                    */
/************************************************************/

#include "DriveCore.h"

#include <cmath>

using namespace std;

namespace canprofile {

string ownerName(Owner owner)
{
  return (owner == OWNER_MANUAL) ? "MANUAL" : "AUTONOMY";
}

static double clampAbs(double v, double lim)
{
  if(std::isnan(v))
    return 0;
  if(v >  lim) return  lim;
  if(v < -lim) return -lim;
  return v;
}

DriveCore::DriveCore()
{
  m_owner      = OWNER_AUTONOMY;
  m_link_ok    = true;
  m_rudder_sp  = 0;
  m_thrust_sp  = 0;
  m_rudder_out = 0;
  m_thrust_req = 0;
  m_gear       = GEAR_NEUTRAL;
  m_gear_time  = -1e9;   // neutral "since forever": first engagement is immediate
  m_last_time  = 0;
  m_first      = true;

  m_rudder_limit = 100;
  m_thrust_limit = 100;
  m_deadband     = 0;
  m_neutral_secs = 0;
  m_settle_secs  = 0;
  m_rudder_rate  = 0;
  m_thrust_rate  = 0;
  m_lost_rudder  = 0;
  m_lost_thrust  = 0;
  m_trim_up_secs   = 1.0;
  m_trim_down_secs = 1.0;
  m_trim         = TRIM_IDLE;
  m_trim_end     = 0;
  m_manual_gear_cmd = false;
  m_gear_sp      = GEAR_NEUTRAL;
}

void DriveCore::configure(const Profile& p)
{
  m_rudder_limit = p.setting("rudder_limit_pct");
  m_thrust_limit = p.setting("thrust_limit_pct");
  m_deadband     = p.setting("thrust_deadband_pct");
  m_neutral_secs = p.setting("shift_neutral_secs");
  m_settle_secs  = p.setting("shift_settle_secs");
  m_rudder_rate  = p.setting("rudder_rate_pct_s");
  m_thrust_rate  = p.setting("thrust_rate_pct_s");
  m_lost_rudder  = p.setting("lost_link_rudder_pct");
  m_lost_thrust  = p.setting("lost_link_thrust_pct");
  m_trim_up_secs   = p.setting("trim_up_secs");
  m_trim_down_secs = p.setting("trim_down_secs");
}

void DriveCore::setOwner(Owner owner)
{
  if(owner == m_owner)
    return;
  m_owner = owner;
  m_rudder_sp = 0;
  m_thrust_sp = 0;
  m_gear_sp = GEAR_NEUTRAL;
  m_trim = TRIM_IDLE;
}

void DriveCore::setGear(Gear gear)
{
  if(gear != m_gear_sp)
    m_thrust_sp = 0;      // never carry throttle across a gear change
  m_gear_sp = gear;
}

void DriveCore::trimCommand(int direction, double now)
{
  if(direction > 0) {
    m_trim = TRIM_UP;
    m_trim_end = now + m_trim_up_secs;
  }
  else if(direction < 0) {
    m_trim = TRIM_DOWN;
    m_trim_end = now + m_trim_down_secs;
  }
  else
    m_trim = TRIM_IDLE;
}

void DriveCore::resetToNeutral(double now)
{
  m_rudder_sp  = 0;
  m_thrust_sp  = 0;
  m_rudder_out = 0;
  m_thrust_req = 0;
  m_trim = TRIM_IDLE;
  m_gear_sp = GEAR_NEUTRAL;
  if(m_gear != GEAR_NEUTRAL) {
    m_gear = GEAR_NEUTRAL;
    m_gear_time = now;
  }
}

double DriveCore::limitRate(double current, double target, double rate, double dt) const
{
  if(rate <= 0)
    return target;
  if(dt <= 0)
    return current;
  double step = rate * dt;
  if(target > current + step) return current + step;
  if(target < current - step) return current - step;
  return target;
}

DriveCommand DriveCore::update(double now)
{
  double dt = m_first ? 0 : (now - m_last_time);
  m_first = false;
  m_last_time = now;

  if(lostLink()) {
    m_rudder_sp = m_lost_rudder;
    m_thrust_sp = m_lost_thrust;
    m_trim = TRIM_IDLE;
    if(m_lost_thrust > 0)      m_gear_sp = GEAR_FWD;
    else if(m_lost_thrust < 0) m_gear_sp = GEAR_REV;
    else                       m_gear_sp = GEAR_NEUTRAL;
  }
  if(m_trim != TRIM_IDLE && now >= m_trim_end)
    m_trim = TRIM_IDLE;   // pulse over

  double rudder_target = clampAbs(m_rudder_sp, m_rudder_limit);
  double thrust_target = clampAbs(m_thrust_sp, m_thrust_limit);

  m_rudder_out = limitRate(m_rudder_out, rudder_target, m_rudder_rate, dt);
  m_thrust_req = limitRate(m_thrust_req, thrust_target, m_thrust_rate, dt);

  // Gear wanted: the operator's gear command in manual (if enabled),
  // otherwise the sign of the thrust request
  Gear wanted = GEAR_NEUTRAL;
  if(gearFromCommand())
    wanted = m_gear_sp;
  else if(m_thrust_req > m_deadband)
    wanted = GEAR_FWD;
  else if(m_thrust_req < -m_deadband)
    wanted = GEAR_REV;

  // Shift interlock. Leaving a gear always goes through neutral; a gear is
  // engaged only after neutral has been held for shift_neutral_secs. With
  // both times at 0 this reduces to "gear follows the sign of thrust".
  for(int i=0; i<2 && wanted != m_gear; i++) {
    if(m_gear != GEAR_NEUTRAL) {
      m_gear = GEAR_NEUTRAL;
      m_gear_time = now;
    }
    else if(now - m_gear_time >= m_neutral_secs) {
      m_gear = wanted;
      m_gear_time = now;
    }
    else
      break;
  }

  DriveCommand cmd;
  cmd.rudder_pct = m_rudder_out;
  cmd.gear = m_gear;
  cmd.trim = m_trim;
  if(m_gear == GEAR_NEUTRAL || (now - m_gear_time) < m_settle_secs)
    cmd.thrust_pct = 0;
  else
    cmd.thrust_pct = (m_gear == GEAR_FWD) ? fabs(m_thrust_req) : -fabs(m_thrust_req);
  return cmd;
}

}  // namespace canprofile
