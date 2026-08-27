/************************************************************/
/*    NAME: Douglas Lima                                    */
/*    FILE: HullModel.h                                     */
/*    DATE: 2026                                            */
/*                                                          */
/*  The USV's motion, ported from uSimMarineV22 so that the */
/*  boat behaves here exactly as it does when the mission    */
/*  runs the simulator inside MOOS:                          */
/*    ~/moos-ivp/ivp/src/uSimMarineV22/SimEngine.cpp         */
/*      propagateSpeed()   :148                              */
/*      propagateHeading() :213                              */
/*    ~/moos-ivp/ivp/src/uSimMarineV22/ThrustMap.cpp         */
/*      getSpeedValuePos()                                   */
/*    ~/moos-ivp/ivp/src/uSimMarineV22/TurnSpeedMap.cpp      */
/*      getTurnRate()                                        */
/************************************************************/

#ifndef AISSIM_HullModel_HEADER
#define AISSIM_HullModel_HEADER

#include <map>
#include <string>

class HullModel
{
 public:
  HullModel();

  /** "0:0, 20:3, 40:6, 60:9, 80:12, 100:15" - thrust percent to metres/second,
   *  the same syntax uSimMarineV22 takes for thrust_map. */
  bool setThrustMap(const std::string& spec);
  void setTurnRate(double v)     { m_turn_rate = v; }
  void setTurnSpdLoss(double v)  { m_turn_spd_loss = v; }
  void setMaxAccel(double v)     { m_max_accel = v; }
  void setMaxDecel(double v)     { m_max_decel = v; }

  void setPose(double x, double y, double heading, double speed);

  /** Advance by dt seconds under the given actuator demand.
   *  rudder is [-100,100] percent, thrust is [0,100] percent - the same units
   *  planchaPID publishes and iCAN consumes on the real boat. */
  void step(double dt, double rudder, double thrust);

  double x() const        { return m_x; }
  double y() const        { return m_y; }
  double heading() const  { return m_heading; }   // degrees true, [0,360)
  double speed() const    { return m_speed; }     // metres/second

 private:
  double speedForThrust(double thrust) const;
  double turnRateForSpeed(double speed) const;

  std::map<double, double> m_thrust_map;   // thrust percent -> m/s

  double m_turn_rate;
  double m_turn_spd_loss;
  double m_max_accel;
  double m_max_decel;

  double m_x, m_y;
  double m_heading;
  double m_speed;
};

#endif
