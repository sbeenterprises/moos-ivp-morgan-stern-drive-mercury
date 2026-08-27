/************************************************************/
/*    NAME: Douglas Lima                                    */
/*    FILE: HullModel.cpp                                   */
/*    DATE: 2026                                            */
/************************************************************/

#include "HullModel.h"
#include <cmath>
#include <cstdlib>
#include <sstream>

using namespace std;

namespace {

double clampd(double v, double lo, double hi)
{
  if(v < lo) return lo;
  if(v > hi) return hi;
  return v;
}

double angle360(double deg)
{
  deg = fmod(deg, 360.0);
  if(deg < 0)
    deg += 360.0;
  return deg;
}

} // anonymous namespace

//---------------------------------------------------------
// Constructor

HullModel::HullModel()
{
  m_turn_rate     = 95;
  m_turn_spd_loss = 0.4;
  m_max_accel     = 0;      // 0 = unlimited, as uSimMarineV22 defaults
  m_max_decel     = 0;

  m_x = 0; m_y = 0;
  m_heading = 0;
  m_speed = 0;
}

//---------------------------------------------------------
// Procedure: setThrustMap()

bool HullModel::setThrustMap(const string& spec)
{
  map<double, double> parsed;

  stringstream ss(spec);
  string pair;
  while(getline(ss, pair, ',')) {
    size_t colon = pair.find(':');
    if(colon == string::npos)
      return false;
    double thrust = atof(pair.substr(0, colon).c_str());
    double speed  = atof(pair.substr(colon + 1).c_str());
    parsed[thrust] = speed;
  }

  if(parsed.empty())
    return false;

  m_thrust_map = parsed;
  return true;
}

//---------------------------------------------------------
// Procedure: setPose()

void HullModel::setPose(double x, double y, double heading, double speed)
{
  m_x = x;
  m_y = y;
  m_heading = angle360(heading);
  m_speed = speed;
}

//---------------------------------------------------------
// Procedure: speedForThrust()
//   Linear interpolation between the mapped points, saturating at the ends -
//   ThrustMap::getSpeedValuePos().

double HullModel::speedForThrust(double thrust) const
{
  if(m_thrust_map.empty() || (thrust <= 0))
    return 0;

  map<double, double>::const_iterator first = m_thrust_map.begin();
  map<double, double>::const_reverse_iterator last = m_thrust_map.rbegin();

  if(thrust >= last->first)
    return last->second;
  if(thrust <= first->first)
    return first->second;

  map<double, double>::const_iterator hi = m_thrust_map.lower_bound(thrust);
  if(hi == m_thrust_map.end())
    return last->second;
  if(hi->first == thrust)
    return hi->second;

  map<double, double>::const_iterator lo = hi;
  --lo;

  double run = hi->first - lo->first;
  if(run <= 0)
    return lo->second;

  double slope = (hi->second - lo->second) / run;
  return ((thrust - lo->first) * slope) + lo->second;
}

//---------------------------------------------------------
// Procedure: turnRateForSpeed()
//   TurnSpeedMap with the defaults propagateHeading() installs: full rate at
//   the configured turn_rate, reached at 5 m/s, zero at zero speed.

double HullModel::turnRateForSpeed(double speed) const
{
  const double full_speed = 5.0;
  if(speed <= 0)
    return 0;
  if(speed >= full_speed)
    return m_turn_rate;
  return (speed / full_speed) * m_turn_rate;
}

//---------------------------------------------------------
// Procedure: step()

void HullModel::step(double dt, double rudder, double thrust)
{
  if(dt <= 0)
    return;

  rudder = clampd(rudder, -100, 100);
  thrust = clampd(thrust, -100, 100);

  // ---- speed, SimEngine::propagateSpeed() ----
  double next_speed = speedForThrust(thrust);

  // A boat in a hard turn loses way.
  double vpct = (fabs(rudder) / 100.0) * m_turn_spd_loss;
  next_speed *= (1.0 - vpct);

  if(next_speed > m_speed) {
    double accel = (next_speed - m_speed) / dt;
    if((m_max_accel > 0) && (accel > m_max_accel))
      next_speed = (m_max_accel * dt) + m_speed;
  }
  else if(next_speed < m_speed) {
    double decel = (m_speed - next_speed) / dt;
    if((m_max_decel > 0) && (decel > m_max_decel))
      next_speed = m_speed - (m_max_decel * dt);
  }

  // ---- heading, SimEngine::propagateHeading() ----
  // Rudder and thruster share an actuator: with the prop stopped, the rudder
  // does nothing. Note this uses the speed from *before* this tick's update,
  // which is what SimEngine does.
  double eff_rudder = (m_speed == 0) ? 0 : rudder;
  double turn_rate  = clampd(turnRateForSpeed(m_speed), 0, 100);

  double delta_deg = eff_rudder * (turn_rate / 100.0) * dt;
  delta_deg = (1 + ((thrust - 50) / 50.0)) * delta_deg;

  m_speed   = next_speed;
  m_heading = angle360(m_heading + delta_deg);

  // ---- position ----
  // MOOS local grid: x is east, y is north, heading is degrees clockwise
  // from north.
  double rad = m_heading * M_PI / 180.0;
  m_x += sin(rad) * m_speed * dt;
  m_y += cos(rad) * m_speed * dt;
}
