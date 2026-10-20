/************************************************************/
/*    FILE: DriveCore.h                                     */
/*    DATE: October 2026                                    */
/*                                                          */
/*    Boat-independent drive behaviour that sits between   */
/*    the MOOS demands and the CAN profile:                 */
/*      - one control owner at a time (manual / autonomy)   */
/*      - setpoints latch until the owner changes them      */
/*      - lost link in manual -> lost_link_* values (0)     */
/*      - rudder/thrust limits and optional rate limits     */
/*      - throttle ramp-up (thrust_rate_up_pct_s): the sent */
/*        throttle only rises gradually; cuts are immediate */
/*      - shift interlock: neutral pause when reversing,    */
/*        throttle held at 0 just after engaging a gear     */
/*      - trim: each command is one short timed pulse       */
/*      - manual gear: in manual control the gear can come  */
/*        from a gear command and thrust is only throttle   */
/************************************************************/

#ifndef DRIVE_CORE_HEADER
#define DRIVE_CORE_HEADER

#include <string>
#include <vector>
#include "CanProfile.h"
#include "AutoTuning.h"

namespace canprofile {

enum Owner { OWNER_AUTONOMY = 0, OWNER_MANUAL = 1 };

std::string ownerName(Owner owner);   // "AUTONOMY" | "MANUAL"

class DriveCore {
 public:
  DriveCore();

  void configure(const Profile& profile);

  // Changing owner clears both setpoints to 0, so a new owner never
  // inherits the previous owner's rudder or throttle.
  void setOwner(Owner owner);
  Owner owner() const { return m_owner; }

  void setRudder(double pct)  { m_rudder_sp = pct; }

  // Boat speed (NAV_SPEED) for the profile's rudder_limit_speed table.
  // The rudder limit then shrinks with speed, in manual and autonomy.
  void setSpeed(double speed, double now);
  double rudderLimitNow() const { return m_rudder_limit_now; }
  bool   speedLimitActive() const { return !m_rudder_speed.empty(); }
  bool   speedStale() const { return m_speed_stale; }
  void setThrust(double pct)  { m_thrust_sp = pct; }

  // Manual gear selection. When enabled (setManualGearFromCommand) and the
  // owner is MANUAL, the gear is whatever setGear() last asked for and the
  // thrust setpoint is throttle only (its sign is ignored); forward or
  // reverse with 0 thrust gives idle in gear. Choosing a different gear
  // resets the thrust setpoint to 0, so the operator re-applies throttle
  // after every shift. In AUTONOMY the gear always follows the thrust sign.
  void setManualGearFromCommand(bool on) { m_manual_gear_cmd = on; }
  bool manualGearFromCommand() const     { return m_manual_gear_cmd; }
  void setGear(Gear gear);
  Gear gearSetpoint() const   { return m_gear_sp; }
  bool gearFromCommand() const { return m_manual_gear_cmd && m_owner == OWNER_MANUAL; }
  double rudderSetpoint() const { return m_rudder_sp; }
  double thrustSetpoint() const { return m_thrust_sp; }

  // Lost link acts while the owner is MANUAL (setLinkOk), and also in
  // AUTONOMY when setLostLinkInAutonomy(true) (iCAN lost_link_autonomy =
  // stop), judged by its own flag (setAutonomyLinkOk) so autonomy can
  // allow a longer outage than manual. The lost-link values replace the
  // setpoints, so the boat stays there when the link comes back until a
  // new command arrives.
  void setLinkOk(bool ok)         { m_link_ok = ok; }
  void setAutonomyLinkOk(bool ok) { m_auto_link_ok = ok; }
  void setLostLinkInAutonomy(bool on) { m_lost_link_autonomy = on; }
  bool lostLinkInAutonomy() const     { return m_lost_link_autonomy; }
  bool lostLink() const {
    if(m_owner == OWNER_MANUAL)
      return !m_link_ok;
    return m_lost_link_autonomy && !m_auto_link_ok;
  }

  // Clears setpoints and returns the gear to neutral (used when CAN
  // sending is switched on again, so it never resumes an old command).
  void resetToNeutral(double now);

  // Run once per tick; returns what should be sent now.
  DriveCommand update(double now);

  Gear currentGear() const { return m_gear; }

  // Trim: +1 = one up pulse, -1 = one down pulse, 0 = stop now.
  // A pulse lasts trim_up_secs / trim_down_secs from the profile (at
  // most 5 s) and then stops by itself. A new command restarts the
  // pulse; it never extends it beyond one pulse length. Trim stops on
  // lost link, owner change and resetToNeutral().
  void trimCommand(int direction, double now);
  void stopTrim()             { m_trim = TRIM_IDLE; }
  Trim currentTrim() const    { return m_trim; }

 private:
  double limitRate(double current, double target, double rate, double dt) const;

 private:
  Owner  m_owner;
  bool   m_link_ok;
  bool   m_lost_link_autonomy;
  bool   m_auto_link_ok;
  double m_rudder_sp;
  double m_thrust_sp;

  double m_rudder_out;
  double m_thrust_req;
  Gear   m_gear;
  double m_gear_time;
  double m_last_time;
  bool   m_first;

  // configuration
  double m_rudder_limit;
  double m_thrust_limit;
  double m_deadband;
  double m_neutral_secs;
  double m_settle_secs;
  double m_rudder_rate;
  double m_thrust_rate;
  double m_thrust_rate_up;     // ramp-up limit on the throttle sent
  double m_thrust_sent;        // |throttle| sent last tick
  double m_lost_rudder;
  double m_lost_thrust;
  double m_trim_up_secs;
  double m_trim_down_secs;

  // speed-dependent rudder limit
  std::vector<SpeedPoint> m_rudder_speed;
  double m_speed_stale_secs;
  double m_speed;
  double m_speed_time;
  bool   m_speed_stale;
  double m_rudder_limit_now;

  Trim   m_trim;
  double m_trim_end;

  bool   m_manual_gear_cmd;
  Gear   m_gear_sp;
};

}  // namespace canprofile

#endif
