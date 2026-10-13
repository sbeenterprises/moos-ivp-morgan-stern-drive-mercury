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
}

void DriveCore::setOwner(Owner owner)
{
  if(owner == m_owner)
    return;
  m_owner = owner;
  m_rudder_sp = 0;
  m_thrust_sp = 0;
}

void DriveCore::resetToNeutral(double now)
{
  m_rudder_sp  = 0;
  m_thrust_sp  = 0;
  m_rudder_out = 0;
  m_thrust_req = 0;
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
  }

  double rudder_target = clampAbs(m_rudder_sp, m_rudder_limit);
  double thrust_target = clampAbs(m_thrust_sp, m_thrust_limit);

  m_rudder_out = limitRate(m_rudder_out, rudder_target, m_rudder_rate, dt);
  m_thrust_req = limitRate(m_thrust_req, thrust_target, m_thrust_rate, dt);

  // Gear wanted by the thrust request
  Gear wanted = GEAR_NEUTRAL;
  if(m_thrust_req > m_deadband)
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
  if(m_gear == GEAR_NEUTRAL || (now - m_gear_time) < m_settle_secs)
    cmd.thrust_pct = 0;
  else
    cmd.thrust_pct = (m_gear == GEAR_FWD) ? fabs(m_thrust_req) : -fabs(m_thrust_req);
  return cmd;
}

}  // namespace canprofile
