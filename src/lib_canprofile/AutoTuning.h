//------------------------------------------------------------------
// AutoTuning: the autopilot (planchaPID) settings of one boat, read
// from the boat profile instead of being hardcoded.
//
// Profile keys (all optional; the defaults are the values planchaPID
// had in code / real.moos before October 2026):
//
//   auto_heading_gains  upto:kp:ki:kd steps, heading PID gains by NAV_SPEED
//   auto_heading_i_authority_pct  most rudder the heading I-term may add
//   auto_heading_limit_pct      clamp of the heading PID output (+/-)
//   auto_rudder_max_pct         rudder limit at or below auto_rudder_speed_low
//   auto_rudder_min_pct         rudder limit at or above auto_rudder_speed_high
//   auto_rudder_speed_low, auto_rudder_speed_high   (NAV_SPEED units)
//   auto_thrust_max_pct         highest DESIRED_THRUST planchaPID sends
//   auto_speed_i_authority_pct  most thrust the speed I-term may add
//   auto_speed_ff     speed:thrust% points, the thrust that holds a speed
//   auto_speed_filter, auto_heading_filter   NAV_SPEED / NAV_HEADING
//                     smoothing, 0 < alpha <= 1 (1 = no smoothing)
//   auto_speed_gains  upto:kp:ki steps, speed PID gains by desired speed
//   auto_speed_ramp_up, auto_speed_ramp_down   most the speed setpoint the
//                     speed loop works to may rise / fall per second
//                     (NAV_SPEED units/s); 0 = follow DESIRED_SPEED at once
//   auto_heading_turn_latch_deg  heading error (deg) above which the turn
//                     direction is held: near a reversal the shortest way
//                     flips between +179 and -179 and the rudder bangs side
//                     to side without turning. 0 = off; else 90..179
//
// Speeds are in NAV_SPEED units (knots on the real boats). Thrust is
// DESIRED_THRUST %, i.e. before the profile's throttle tx_field map.
//------------------------------------------------------------------

#ifndef CANPROFILE_AUTOTUNING_H
#define CANPROFILE_AUTOTUNING_H

#include <string>
#include <vector>

namespace canprofile {

class Profile;

struct SpeedPoint { double speed; double thrust; };
struct GainStep   { double up_to; double kp; double ki; double kd; };

class AutoTuning {
 public:
  AutoTuning();   // the old hardcoded values

  // Takes every auto_* value from a valid profile. Returns false (and
  // leaves the object unchanged) if a map does not parse.
  bool configure(const Profile& p, std::string* err = 0);

  // Map parsers, also used by Profile validation
  static bool parseSpeedFF(const std::string& s, std::vector<SpeedPoint>& out,
                           std::string& err);
  // speed:percent points, speeds increasing, percent 0..100; "key" names
  // the profile key in error messages. Also used for rudder_limit_speed.
  static bool parseSpeedPct(const std::string& s, const std::string& key,
                            std::vector<SpeedPoint>& out, std::string& err);
  static double interpSpeed(const std::vector<SpeedPoint>& pts, double speed);
  static bool parseGains(const std::string& s, std::vector<GainStep>& out,
                         std::string& err);
  static bool parseHeadingGains(const std::string& s, std::vector<GainStep>& out,
                                std::string& err);

  double feedForward(double desired_speed) const;               // thrust %
  void   gains(double desired_speed, double& kp, double& ki) const;
  void   headingGains(double speed, double& kp, double& ki, double& kd) const;
  double rudderLimit(double speed) const;                       // +/- rudder %

  double heading_limit, heading_i_authority;
  double rudder_max, rudder_min, rudder_speed_low, rudder_speed_high;
  double thrust_max, i_authority;
  double speed_filter, heading_filter;
  double speed_ramp_up, speed_ramp_down;
  double heading_turn_latch;   // deg, 0 = off
  std::vector<SpeedPoint> speed_ff;
  std::vector<GainStep>   speed_gains;
  std::vector<GainStep>   heading_gains;
};

// Default map strings, shared with the Profile defaults
extern const char* const DEFAULT_AUTO_SPEED_FF;
extern const char* const DEFAULT_AUTO_SPEED_GAINS;
extern const char* const DEFAULT_AUTO_HEADING_GAINS;

}  // namespace canprofile

#endif
