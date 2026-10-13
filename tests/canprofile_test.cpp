//------------------------------------------------------------------
// Tests for lib_canprofile (profile parsing, frame encoding, feedback
// decoding, validation and DriveCore behaviour).
//
// Build and run:   tests/run_tests.sh
// Needs only g++; no MOOS, no CAN hardware.
//------------------------------------------------------------------

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "CanProfile.h"
#include "DriveCore.h"

using namespace std;
using namespace canprofile;

// Old iCAN frame builders (legacy_mercury_frames.cpp)
string legacyRudderFrame(double rudder_value);
string legacyGearFrame(double thrust_value);
string legacyThrustFrame(double thrust_value);

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond, what)                                              \
  do {                                                                 \
    g_checks++;                                                        \
    if(!(cond)) {                                                      \
      g_failures++;                                                    \
      cout << "FAIL " << __LINE__ << ": " << what << endl;             \
    }                                                                  \
  } while(0)

static string g_profile_dir;

static const TxMsg* msgNamed(const Profile& p, const string& name)
{
  for(size_t i=0; i<p.txMessages().size(); i++)
    if(p.txMessages()[i].name == name)
      return &p.txMessages()[i];
  return 0;
}

static Gear legacyGear(double thrust)
{
  if(thrust > 0) return GEAR_FWD;
  if(thrust == 0) return GEAR_NEUTRAL;
  return GEAR_REV;
}

static string fmt(double v)
{
  char buf[32];
  snprintf(buf, sizeof(buf), "%g", v);
  return buf;
}

static void printErrors(const Profile& p)
{
  for(size_t i=0; i<p.errors().size(); i++)
    cout << "    " << p.errors()[i] << endl;
}

//------------------------------------------------------------------
// Mercury: every frame must equal the old hardcoded tables

static void testMercuryMatchesLegacy()
{
  Profile p;
  bool ok = p.load(g_profile_dir + "/mercury_sterndrive.prf");
  CHECK(ok, "mercury profile loads without errors");
  if(!ok) { printErrors(p); return; }

  const TxMsg* steer = msgNamed(p, "steer");
  const TxMsg* gear  = msgNamed(p, "gear");
  const TxMsg* tsc1  = msgNamed(p, "tsc1");
  CHECK(steer && gear && tsc1, "mercury has steer, gear and tsc1 messages");
  if(!steer || !gear || !tsc1) return;

  int compared = 0;
  for(int i=-600; i<=600; i++) {
    double v = i * 0.25;    // -150 .. 150 in quarter steps
    DriveCommand cmd;
    cmd.rudder_pct = v;
    string got = p.encode(*steer, cmd).str();
    string want = legacyRudderFrame(v);
    CHECK(got == want, "rudder " << fmt(v) << ": got " << got << " want " << want);

    cmd = DriveCommand();
    cmd.thrust_pct = v;
    cmd.gear = legacyGear(v);
    got = p.encode(*gear, cmd).str();
    want = legacyGearFrame(v);
    CHECK(got == want, "gear for thrust " << fmt(v) << ": got " << got << " want " << want);
    got = p.encode(*tsc1, cmd).str();
    want = legacyThrustFrame(v);
    CHECK(got == want, "tsc1 for thrust " << fmt(v) << ": got " << got << " want " << want);
    compared += 3;
  }

  // Small values that the old code truncated to centre / idle
  double odd[] = {-0.999, -0.5, -0.001, 0.001, 0.5, 0.999, 99.999, -99.999};
  for(int i=0; i<8; i++) {
    DriveCommand cmd;
    cmd.rudder_pct = odd[i];
    CHECK(p.encode(*steer, cmd).str() == legacyRudderFrame(odd[i]),
          "rudder " << odd[i] << " matches legacy");
    cmd.thrust_pct = odd[i];
    cmd.gear = legacyGear(odd[i]);
    CHECK(p.encode(*tsc1, cmd).str() == legacyThrustFrame(odd[i]),
          "tsc1 " << odd[i] << " matches legacy");
    CHECK(p.encode(*gear, cmd).str() == legacyGearFrame(odd[i]),
          "gear " << odd[i] << " matches legacy");
  }
  cout << "  mercury: compared " << compared + 24 << " frames with the old code" << endl;

  // The same through DriveCore (no shift pauses on this boat), so the
  // whole path from DESIRED_THRUST to frame is unchanged.
  DriveCore core;
  core.configure(p);
  double t = 0;
  for(int i=-100; i<=100; i+=7) {
    core.setThrust(i);
    DriveCommand cmd = core.update(t += 0.05);
    CHECK(p.encode(*gear, cmd).str() == legacyGearFrame(i), "core gear for thrust " << i);
    CHECK(p.encode(*tsc1, cmd).str() == legacyThrustFrame(i), "core tsc1 for thrust " << i);
  }
}

//------------------------------------------------------------------
// USV-722: frames must match the CAN ID mapping workbook (9 Oct 2026)

static void testUsv722Frames()
{
  Profile p;
  bool ok = p.load(g_profile_dir + "/usv722_glendinning.prf");
  CHECK(ok, "usv722 profile loads without errors");
  if(!ok) { printErrors(p); return; }

  const TxMsg* steer = msgNamed(p, "steer");
  const TxMsg* gear  = msgNamed(p, "gear");
  const TxMsg* thr   = msgNamed(p, "throttle");
  CHECK(steer && gear && thr, "usv722 has steer, gear and throttle messages");
  if(!steer || !gear || !thr) return;

  // "Percent Lookup" sheet: percentage -> raw steering value
  int steer_raw[41] = {0, 26, 51, 77, 102, 128, 153, 179, 204, 230, 256, 281, 307,
                       332, 358, 383, 409, 434, 460, 485, 511, 536, 561, 586, 611,
                       636, 661, 686, 711, 736, 762, 787, 812, 837, 862, 887, 912,
                       937, 962, 987, 1012};
  for(int i=0; i<41; i++) {
    int pct = -100 + 5 * i;
    DriveCommand cmd;
    cmd.rudder_pct = pct;
    Frame f = p.encode(*steer, cmd);
    char want[64];
    snprintf(want, sizeof(want), "18FF11D0#%02X%02XFFFFFFFFFFFF",
             steer_raw[i] & 0xFF, steer_raw[i] >> 8);
    CHECK(f.str() == want, "steer " << pct << "%: got " << f.str() << " want " << want);
  }

  // "Throttle & Gear Lookup" sheet: throttle % -> byte 1
  int thr_byte[21] = {0, 13, 25, 38, 50, 63, 75, 88, 100, 113, 125, 138, 150,
                      163, 175, 188, 200, 213, 225, 238, 250};
  for(int i=0; i<21; i++) {
    int pct = 5 * i;
    DriveCommand cmd;
    cmd.thrust_pct = pct;
    cmd.gear = GEAR_FWD;
    char want[64];
    snprintf(want, sizeof(want), "0CF003D0#FF%02XFFFFFFFFFFFF", thr_byte[i]);
    CHECK(p.encode(*thr, cmd).str() == want, "throttle " << pct << "% -> " << want);
    cmd.thrust_pct = -pct;     // same lever value in reverse
    cmd.gear = GEAR_REV;
    CHECK(p.encode(*thr, cmd).str() == want, "throttle -" << pct << "% -> " << want);
  }

  DriveCommand cmd;
  cmd.gear = GEAR_FWD;
  CHECK(p.encode(*gear, cmd).str() == "18F005D0#7EFFFFFFFFFFFFFF", "gear forward 7E");
  cmd.gear = GEAR_NEUTRAL;
  CHECK(p.encode(*gear, cmd).str() == "18F005D0#7DFFFFFFFFFFFFFF", "gear neutral 7D");
  cmd.gear = GEAR_REV;
  CHECK(p.encode(*gear, cmd).str() == "18F005D0#7CFFFFFFFFFFFFFF", "gear reverse 7C");

  // Out-of-range commands are clamped, never wrapped
  cmd.rudder_pct = 250;
  CHECK(p.encode(*steer, cmd).str() == "18FF11D0#F403FFFFFFFFFFFF", "steer clamps at 1012");
  cmd.rudder_pct = -250;
  CHECK(p.encode(*steer, cmd).str() == "18FF11D0#0000FFFFFFFFFFFF", "steer clamps at 0");
  cmd.rudder_pct = NAN;
  CHECK(p.encode(*steer, cmd).str() == "18FF11D0#FF01FFFFFFFFFFFF", "NaN steers to centre");
}

static double rxValue(const Profile& p, uint32_t id, bool ext,
                      const vector<uint8_t>& data, const string& name, string* sval = 0)
{
  vector<RxValue> vals = p.decode(id, ext, &data[0], (int)data.size());
  for(size_t i=0; i<vals.size(); i++)
    if(vals[i].name == name) {
      if(sval) *sval = vals[i].sval;
      return vals[i].dval;
    }
  return -9999;
}

static void testUsv722Feedback()
{
  Profile p;
  if(!p.load(g_profile_dir + "/usv722_glendinning.prf"))
    return;

  // Frames from the "Send Test Feedback" sheet
  uint8_t a[] = {0x00, 0xFF, 0x7F, 0x7D, 0x00, 0x64, 0x55};   // cmd 383, actual 381
  vector<uint8_t> f1(a, a + 7);
  double actual = rxValue(p, 0x210, false, f1, "CAN_RUDDER_ACTUAL_PCT");
  double echo   = rxValue(p, 0x210, false, f1, "CAN_RUDDER_ECHO_PCT");
  CHECK(fabs(actual - (381 - 511) * 100.0 / 511) < 1e-9, "actual 381 -> " << actual);
  CHECK(fabs(echo   - (383 - 511) * 100.0 / 511) < 1e-9, "echo 383 -> " << echo);

  uint8_t b[] = {0x00, 0xFF, 0x7C, 0x7F, 0x00, 0x64, 0x5A};   // cmd 636, actual 639
  vector<uint8_t> f2(b, b + 7);
  actual = rxValue(p, 0x210, false, f2, "CAN_RUDDER_ACTUAL_PCT");
  echo   = rxValue(p, 0x210, false, f2, "CAN_RUDDER_ECHO_PCT");
  CHECK(fabs(actual - (639 - 511) * 100.0 / 501) < 1e-9, "actual 639 -> " << actual);
  CHECK(fabs(echo   - (636 - 511) * 100.0 / 501) < 1e-9, "echo 636 -> " << echo);

  uint8_t c[] = {0x00, 0xFF, 0x90, 0xD9, 0x1E, 0x64, 0x55};   // motor drive 30 %
  vector<uint8_t> f3(c, c + 7);
  CHECK(rxValue(p, 0x210, false, f3, "CAN_STEER_MOTOR_PCT") == 30, "motor drive 30%");

  // The same ID as a 29-bit frame must not match the 11-bit feedback
  CHECK(p.decode(0x210, true, a, 7).empty(), "29-bit 0x210 is ignored");

  string s;
  uint8_t h1[] = {0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  rxValue(p, 0x18FF08D0, true, vector<uint8_t>(h1, h1 + 8), "CAN_HELM_HAS_STEERING", &s);
  CHECK(s == "true", "18FF08D0 F0 -> helm has steering");
  uint8_t h2[] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  rxValue(p, 0x18FF08D0, true, vector<uint8_t>(h2, h2 + 8), "CAN_HELM_HAS_STEERING", &s);
  CHECK(s == "false", "18FF08D0 00 -> PC has steering");

  uint8_t g[] = {0x7C, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  rxValue(p, 0x00F005E2, true, vector<uint8_t>(g, g + 8), "CAN_ENGINE_GEAR", &s);
  CHECK(s == "rev", "00F005E2 7C -> rev");

  uint8_t t[] = {0xFF, 0x7B, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  CHECK(fabs(rxValue(p, 0x0CF003D0, true, vector<uint8_t>(t, t + 8),
                     "CAN_HELM_THROTTLE_PCT") - 49.2) < 1e-9, "throttle byte 7B -> 49.2%");

  uint8_t fu[] = {0x00, 0x1E, 0x16, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  CHECK(fabs(rxValue(p, 0x19F21170, true, vector<uint8_t>(fu, fu + 8),
                     "CAN_FUEL_PCT") - 22.648) < 1e-9, "fuel 5662 -> 22.648%");

  string name;
  CHECK(p.isTxId(0x18FF11D0, true, name) && name == "steer", "18FF11D0 is our steer ID");
  CHECK(!p.isTxId(0x00F003E2, true, name), "00F003E2 is not one of our IDs");
}

//------------------------------------------------------------------
// Validation: a bad profile must be rejected with a clear error

static bool rejects(const string& body, const string& expect_text)
{
  string base =
    "profile = t\ncan_interface = vcan0\ncapabilities = steer\n"
    "tx_msg = name=steer, cap=steer, id=18FF11D0, dlc=8, period_ms=100\n";
  Profile p;
  p.loadFromString(base + body, g_profile_dir);
  if(p.valid())
    return false;
  for(size_t i=0; i<p.errors().size(); i++)
    if(p.errors()[i].find(expect_text) != string::npos)
      return true;
  cout << "    errors were:" << endl;
  printErrors(p);
  return false;
}

static void testValidation()
{
  string field = "tx_field = msg=steer, src=rudder_pct, byte=0, bits=16, map=points, points=-100:0|0:511|100:1012\n";

  Profile good;
  good.loadFromString(
    "profile = t\ncan_interface = vcan0\ncapabilities = steer\n"
    "tx_msg = name=steer, cap=steer, id=18FF11D0, dlc=8, period_ms=100\n" + field,
    g_profile_dir);
  CHECK(good.valid(), "minimal profile is valid");
  printErrors(good);

  CHECK(rejects("", "has no tx_field"), "message without fields");
  CHECK(rejects(field + "tx_field = msg=steer, src=rudder_pct, byte=1, bits=8, map=linear\n",
                "written twice"), "overlapping fields");
  CHECK(rejects("tx_field = msg=steer, src=rudder_pct, byte=7, bits=16, map=linear\n",
                "runs past dlc"), "field past the end of the frame");
  CHECK(rejects("tx_field = msg=steer, src=rudder_pct, byte=0, bits=8, map=points, points=-100:0|100:300\n",
                "does not fit"), "value too large for 8 bits");
  CHECK(rejects("tx_field = msg=steer, src=rudder_pct, byte=0, bits=8, map=points, points=-50:0|100:200\n",
                "must cover"), "points not covering -100..100");
  CHECK(rejects(field + "tx_field = msg=steer, src=rudder_side, byte=2, bits=8, map=enum, values=port:1|stbd:2\n",
                "no value for 'center'"), "enum missing a state");
  CHECK(rejects(field + "tx_field = msg=steer, src=gear, byte=3, bits=8, map=enum, values=fwd:1|neutral:2|rev:3\n",
                "does not belong"), "gear field in a steer message");
  CHECK(rejects(field + "rudder_limit = 50\n", "unknown setting"), "misspelled setting");
  CHECK(rejects(field + "capabilities = steer, thrust\n", "no tx_msg has cap=thrust"),
        "capability without a message");
  CHECK(rejects(field + "tx_msg = name=dup, cap=steer, id=18FF11D0, dlc=8, period_ms=100\n"
                "tx_field = msg=dup, src=rudder_pct, byte=0, bits=16, map=linear\n",
                "same id"), "two messages with the same id");
  CHECK(rejects(field + "tx_field = msg=nosuch, src=rudder_pct, byte=0, bits=8, map=linear\n",
                "no tx_msg named"), "field for an undefined message");
  CHECK(rejects("tx_field = msg=steer, src=rudder_pct, byte=0, bits=16, map=table, table=tables/does_not_exist.csv\n",
                "cannot read table"), "missing table file");
  CHECK(rejects(field + "lost_link_rudder_pct = 20\nrudder_limit_pct = 10\n",
                "outside rudder_limit_pct"), "lost-link value beyond the limit");
  CHECK(rejects(field + "rudder_feedback = NOPE\n", "no rx_field"), "unknown feedback name");

  Profile missing;
  CHECK(!missing.load(g_profile_dir + "/no_such_boat.prf"), "missing profile file is rejected");
}

//------------------------------------------------------------------
// DriveCore: latch, owner change, lost link, limits, shift interlock

static void testDriveCore()
{
  Profile p;
  p.load(g_profile_dir + "/usv722_glendinning.prf");
  DriveCore core;
  core.configure(p);   // shift_neutral_secs 1.0, settle 0.5

  // Manual setpoints are held until the operator changes them
  core.setOwner(OWNER_MANUAL);
  core.setRudder(-20);
  double t = 0;
  DriveCommand cmd;
  for(int i=0; i<200; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.rudder_pct == -20, "manual rudder -20 is held after 10 s, got " << cmd.rudder_pct);

  // Gear engages at once from neutral; throttle waits shift_settle_secs
  core.setThrust(30);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_FWD && cmd.thrust_pct == 0, "forward engaged, throttle held at 0");
  for(int i=0; i<10; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_FWD && cmd.thrust_pct == 30, "throttle 30 after settle, got " << cmd.thrust_pct);
  for(int i=0; i<200; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.thrust_pct == 30 && cmd.rudder_pct == -20, "manual throttle 30 is held");

  // Reversing goes through neutral for shift_neutral_secs
  core.setThrust(-30);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_NEUTRAL && cmd.thrust_pct == 0, "reverse request: neutral first");
  double t_neutral = t;
  while(t < t_neutral + 0.95) {
    cmd = core.update(t += 0.05);
    CHECK(cmd.gear == GEAR_NEUTRAL, "still neutral at +" << fmt(t - t_neutral) << " s");
  }
  for(int i=0; i<3; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_REV && cmd.thrust_pct == 0, "reverse engaged, throttle held at 0");
  for(int i=0; i<12; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_REV && cmd.thrust_pct == -30, "reverse throttle 30 after settle");

  // Lost link in manual: rudder 0 and throttle 0 ...
  core.setLinkOk(false);
  cmd = core.update(t += 0.05);
  CHECK(core.lostLink(), "lost link reported");
  CHECK(cmd.rudder_pct == 0 && cmd.thrust_pct == 0 && cmd.gear == GEAR_NEUTRAL,
        "lost link -> rudder 0, throttle 0, neutral");
  // ... and it stays there when the link comes back
  core.setLinkOk(true);
  for(int i=0; i<40; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.rudder_pct == 0 && cmd.thrust_pct == 0, "stays at 0 after the link returns");
  core.setRudder(15);
  cmd = core.update(t += 0.05);
  CHECK(cmd.rudder_pct == 15, "new operator command is accepted after link returns");

  // Owner change clears the setpoints
  core.setOwner(OWNER_AUTONOMY);
  cmd = core.update(t += 0.05);
  CHECK(cmd.rudder_pct == 0 && core.thrustSetpoint() == 0, "owner change clears setpoints");

  // Lost link does nothing while autonomy owns the boat
  core.setRudder(10);
  core.setLinkOk(false);
  cmd = core.update(t += 0.05);
  CHECK(!core.lostLink() && cmd.rudder_pct == 10, "lost link ignored in autonomy");
  core.setLinkOk(true);

  // Limits
  Profile lim;
  lim.loadFromString(
    "profile = t\ncan_interface = vcan0\ncapabilities = steer\nrudder_limit_pct = 90\n"
    "rudder_rate_pct_s = 40\n"
    "tx_msg = name=steer, cap=steer, id=18FF11D0, dlc=8, period_ms=100\n"
    "tx_field = msg=steer, src=rudder_pct, byte=0, bits=16, map=points, points=-100:0|0:511|100:1012\n",
    g_profile_dir);
  CHECK(lim.valid(), "limit test profile is valid");
  DriveCore c2;
  c2.configure(lim);
  c2.setRudder(100);
  t = 0;
  cmd = c2.update(t);
  cmd = c2.update(t += 1.0);
  CHECK(fabs(cmd.rudder_pct - 40) < 1e-9, "rate limit 40 %/s, got " << cmd.rudder_pct);
  for(int i=0; i<10; i++)
    cmd = c2.update(t += 1.0);
  CHECK(cmd.rudder_pct == 90, "rudder limited to 90, got " << cmd.rudder_pct);

  // resetToNeutral (CAN sending switched back on)
  core.setRudder(30);
  core.setThrust(40);
  core.update(t += 1);
  core.resetToNeutral(t);
  cmd = core.update(t += 0.05);
  CHECK(cmd.rudder_pct == 0 && cmd.thrust_pct == 0 && cmd.gear == GEAR_NEUTRAL,
        "reset returns to neutral");
}

//------------------------------------------------------------------

int main(int argc, char* argv[])
{
  g_profile_dir = (argc > 1) ? argv[1] : "profiles";

  cout << "lib_canprofile tests (profiles from " << g_profile_dir << ")" << endl;
  testMercuryMatchesLegacy();
  testUsv722Frames();
  testUsv722Feedback();
  testValidation();
  testDriveCore();

  cout << g_checks << " checks, " << g_failures << " failed" << endl;
  return g_failures ? 1 : 0;
}
