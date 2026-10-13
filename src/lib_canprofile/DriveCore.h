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
/*      - shift interlock: neutral pause when reversing,    */
/*        throttle held at 0 just after engaging a gear     */
/*      - trim: each command is one short timed pulse       */
/************************************************************/

#ifndef DRIVE_CORE_HEADER
#define DRIVE_CORE_HEADER

#include <string>
#include "CanProfile.h"

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
  void setThrust(double pct)  { m_thrust_sp = pct; }
  double rudderSetpoint() const { return m_rudder_sp; }
  double thrustSetpoint() const { return m_thrust_sp; }

  // Lost link only acts while the owner is MANUAL. The lost-link values
  // replace the setpoints, so the boat stays there when the link comes
  // back until the operator sends a new command.
  void setLinkOk(bool ok)     { m_link_ok = ok; }
  bool lostLink() const       { return m_owner == OWNER_MANUAL && !m_link_ok; }

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
  double m_lost_rudder;
  double m_lost_thrust;
  double m_trim_up_secs;
  double m_trim_down_secs;

  Trim   m_trim;
  double m_trim_end;
};

}  // namespace canprofile

#endif
