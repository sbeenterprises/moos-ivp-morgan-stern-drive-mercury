//------------------------------------------------------------------
// Tests for lib_canprofile (profile parsing, frame encoding, feedback
// decoding, validation and DriveCore behaviour).
//
// Build and run:   tests/run_tests.sh
// Needs only g++; no MOOS, no CAN hardware.
//------------------------------------------------------------------

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "CanProfile.h"
#include "DriveCore.h"
#include "AutoTuning.h"

using namespace std;
using namespace canprofile;

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

//------------------------------------------------------------------
// Mercury (USV-725): frames must match the CAN ID mapping (15 Oct 2026)

static void testMercuryFrames()
{
  Profile p;
  bool ok = p.load(g_profile_dir + "/mercury_sterndrive.prf");
  CHECK(ok, "mercury profile loads without errors");
  if(!ok) { printErrors(p); return; }

  const TxMsg* steer = msgNamed(p, "steer");
  const TxMsg* gear  = msgNamed(p, "gear");
  const TxMsg* thr   = msgNamed(p, "throttle");
  CHECK(steer && gear && thr, "mercury has steer, gear and throttle messages");
  if(!steer || !gear || !thr) return;

  // Only the four PC commands the boat accepts, all guarded. Never
  // TSC1 (SA 23 owns engine speed) or the steering controller's frame.
  CHECK(p.txMessages().size() == 4, "mercury sends exactly four messages");
  string name;
  CHECK(!p.isTxId(0x18000017, true, name), "mercury never sends TSC1");
  CHECK(!p.isTxId(0x18F30A1A, true, name), "mercury never sends 18F30A1A");
  for(size_t i=0; i<p.txMessages().size(); i++)
    CHECK(p.txMessages()[i].guard, "mercury " << p.txMessages()[i].name << " is guarded");

  DriveCommand cmd;
  cmd.rudder_pct = -100;
  CHECK(p.encode(*steer, cmd).str() == "18FF11D0#0000FFFFFFFFFFFF", "steer full port 0");
  cmd.rudder_pct = 0;
  CHECK(p.encode(*steer, cmd).str() == "18FF11D0#FF01FFFFFFFFFFFF", "steer centre 511");
  cmd.rudder_pct = 100;
  CHECK(p.encode(*steer, cmd).str() == "18FF11D0#F403FFFFFFFFFFFF", "steer full starboard 1012");

  // Throttle sweep: byte 1 = % / 0.4, same value ahead and astern
  int pct[5]  = {0, 25, 50, 75, 100};
  int byte[5] = {0x00, 0x3F, 0x7D, 0xBC, 0xFA};   // x2.5, rounded
  for(int i=0; i<5; i++) {
    char want[64];
    snprintf(want, sizeof(want), "0CF003D0#FF%02XFFFFFFFFFFFF", byte[i]);
    cmd = DriveCommand();
    cmd.gear = GEAR_FWD;
    cmd.thrust_pct = pct[i];
    CHECK(p.encode(*thr, cmd).str() == want, "throttle " << pct[i] << "% -> " << want);
    cmd.gear = GEAR_REV;
    cmd.thrust_pct = -pct[i];
    CHECK(p.encode(*thr, cmd).str() == want, "throttle -" << pct[i] << "% -> " << want);
  }

  cmd = DriveCommand();
  cmd.gear = GEAR_FWD;
  CHECK(p.encode(*gear, cmd).str() == "18F005D0#7EFFFFFFFFFFFFFF", "gear forward 7E");
  cmd.gear = GEAR_NEUTRAL;
  CHECK(p.encode(*gear, cmd).str() == "18F005D0#7DFFFFFFFFFFFFFF", "gear neutral 7D");
  cmd.gear = GEAR_REV;
  CHECK(p.encode(*gear, cmd).str() == "18F005D0#7CFFFFFFFFFFFFFF", "gear reverse 7C");

  // Manual: forward gear with no throttle sends 7E and throttle 0
  DriveCore core;
  core.configure(p);
  core.setManualGearFromCommand(true);
  core.setOwner(OWNER_MANUAL);
  core.setGear(GEAR_FWD);
  cmd = core.update(1.0);
  CHECK(p.encode(*gear, cmd).str() == "18F005D0#7EFFFFFFFFFFFFFF", "manual forward gear frame");
  CHECK(p.encode(*thr, cmd).str() == "0CF003D0#FF00FFFFFFFFFFFF", "manual forward, throttle 0");

  // Feedback, frames from the 15 Oct captures
  uint8_t sd[] = {0xD9, 0x6A, 0xFF, 0xFF, 0xFF, 0xFF, 0xE5, 0x01};    // -39 = full port
  CHECK(fabs(rxValue(p, 0x18F30A1A, true, vector<uint8_t>(sd, sd + 8),
                     "CAN_STEER_DEMAND_PCT") + 100) < 1e-6, "steer demand -39 -> -100%");
  uint8_t th[] = {0xFF, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  CHECK(fabs(rxValue(p, 0x0CF003D0, true, vector<uint8_t>(th, th + 8),
                     "CAN_HELM_THROTTLE_PCT") - 100) < 1e-9, "helm throttle FA -> 100%");
  string s;
  uint8_t ge[] = {0x7D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  rxValue(p, 0x18F005D0, true, vector<uint8_t>(ge, ge + 8), "CAN_HELM_GEAR", &s);
  CHECK(s == "neutral", "helm gear 7D -> neutral");
  uint8_t ts[] = {0xC1, 0x40, 0x83, 0xE1, 0xFF, 0xFF, 0xFF, 0xFF};
  CHECK(rxValue(p, 0x18000017, true, vector<uint8_t>(ts, ts + 8),
                "CAN_ENGINE_RPM_REQUEST") == 4200, "TSC1 40 83 -> 4200 rpm");
}

//------------------------------------------------------------------
// map=table: input truncated, looked up in a CSV next to the profile

static void testTableMap()
{
  string dir = "/tmp";
  if(getenv("TMPDIR")) dir = getenv("TMPDIR");
  string csv = dir + "/canprofile_table_test.csv";
  FILE* f = fopen(csv.c_str(), "w");
  CHECK(f != 0, "can write a table file");
  if(!f) return;
  fprintf(f, "# input,raw\n");
  for(int i=-100; i<=100; i++)
    fprintf(f, "%d,%d\n", i, 1000 + 10 * i);
  fclose(f);

  Profile p;
  p.loadFromString(
    "profile = t\ncan_interface = vcan0\ncapabilities = steer\n"
    "tx_msg = name=steer, cap=steer, id=18FF11D0, dlc=8, period_ms=100\n"
    "tx_field = msg=steer, src=rudder_pct, byte=0, bits=16, order=be, map=table, "
    "table=canprofile_table_test.csv\n", dir);
  remove(csv.c_str());
  CHECK(p.valid(), "table profile is valid");
  if(!p.valid()) { printErrors(p); return; }

  const TxMsg* steer = msgNamed(p, "steer");
  DriveCommand cmd;
  cmd.rudder_pct = 0;
  CHECK(p.encode(*steer, cmd).str() == "18FF11D0#03E8FFFFFFFFFFFF", "table 0 -> 1000: got " << p.encode(*steer, cmd).str());
  cmd.rudder_pct = 25.9;     // truncated to 25
  CHECK(p.encode(*steer, cmd).str() == "18FF11D0#04E2FFFFFFFFFFFF", "table 25.9 -> 1250: got " << p.encode(*steer, cmd).str());
  cmd.rudder_pct = -100;
  CHECK(p.encode(*steer, cmd).str() == "18FF11D0#0000FFFFFFFFFFFF", "table -100 -> 0: got " << p.encode(*steer, cmd).str());
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

  // Lever curve (16 Oct 2026 logs): lever % -> engine byte, same astern.
  // lever 0/10/40/70/100 -> engine 0/30/40/45/50 % (byte = engine x 2.5)
  int lever[5] = {0, 10, 40, 70, 100};
  int ebyte[5] = {0x00, 0x4B, 0x64, 0x71, 0x7D};
  for(int i=0; i<5; i++) {
    DriveCommand cmd;
    cmd.thrust_pct = lever[i];
    cmd.gear = GEAR_FWD;
    char want[64];
    snprintf(want, sizeof(want), "0CF003D0#FF%02XFFFFFFFFFFFF", ebyte[i]);
    CHECK(p.encode(*thr, cmd).str() == want, "lever " << lever[i] << "% -> " << want
          << " got " << p.encode(*thr, cmd).str());
    cmd.thrust_pct = -lever[i];     // same lever value in reverse
    cmd.gear = GEAR_REV;
    CHECK(p.encode(*thr, cmd).str() == want, "lever -" << lever[i] << "% -> " << want);
  }
  {
    // Never above engine 50 % (byte 7D), even past 100 %
    DriveCommand cmd;
    cmd.gear = GEAR_FWD;
    bool capped = true;
    for(int v=0; v<=150; v++) {
      cmd.thrust_pct = v;
      string f = p.encode(*thr, cmd).str();
      if(strtol(f.substr(11, 2).c_str(), 0, 16) > 0x7D) capped = false;
    }
    CHECK(capped, "722 throttle never above engine 50 %");
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

// The USV-722 profile without its rudder speed limit, rudder slew and
// throttle ramp-up, for tests of the other DriveCore behaviour (those
// have their own tests).
static void loadUsv722NoRudderShaping(Profile& p)
{
  ifstream in((g_profile_dir + "/usv722_glendinning.prf").c_str());
  string line, text;
  while(getline(in, line))
    if(line.find("rudder_limit_speed") != 0 && line.find("rudder_rate_pct_s") != 0 &&
       line.find("thrust_rate_up_pct_s") != 0)
      text += line + "\n";
  p.loadFromString(text, g_profile_dir);
}

static void testDriveCore()
{
  Profile p;
  loadUsv722NoRudderShaping(p);
  CHECK(p.valid(), "usv722 profile (no rudder shaping) loads");
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
// Trim: frames per boat, timed pulses, stops and validation

static void testTrim()
{
  // Mercury: PC trim command from the USV-725 CAN ID mapping
  Profile m;
  CHECK(m.load(g_profile_dir + "/mercury_sterndrive.prf"), "mercury loads with trim message");
  const TxMsg* mt = msgNamed(m, "trim");
  CHECK(mt != 0, "mercury defines a trim message");
  CHECK(m.hasCapability("trim"), "mercury trim is enabled");
  if(mt) {
    DriveCommand cmd;
    cmd.trim = TRIM_UP;
    CHECK(m.encode(*mt, cmd).str() == "18FF08D0#F1FFFFFFFFFFFFFF", "mercury trim up F1");
    cmd.trim = TRIM_DOWN;
    CHECK(m.encode(*mt, cmd).str() == "18FF08D0#F4FFFFFFFFFFFFFF", "mercury trim down F4");
    cmd.trim = TRIM_IDLE;
    CHECK(m.encode(*mt, cmd).str() == "18FF08D0#F0FFFFFFFFFFFFFF", "mercury trim idle F0");
    CHECK(m.hasTrimIdle(*mt), "mercury has an idle frame");
    CHECK(mt->guard, "mercury trim is guarded");
  }

  // USV-722: trim switch byte from the workbook
  Profile u;
  CHECK(u.load(g_profile_dir + "/usv722_glendinning.prf"), "usv722 loads with trim message");
  const TxMsg* ut = msgNamed(u, "trim");
  CHECK(ut != 0 && u.hasCapability("trim"), "usv722 defines trim, enabled");
  if(ut) {
    DriveCommand cmd;
    cmd.trim = TRIM_UP;
    CHECK(u.encode(*ut, cmd).str() == "18FF08D0#F1FFFFFFFFFFFFFF", "usv722 trim up F1");
    cmd.trim = TRIM_DOWN;
    CHECK(u.encode(*ut, cmd).str() == "18FF08D0#F4FFFFFFFFFFFFFF", "usv722 trim down F4");
    cmd.trim = TRIM_IDLE;
    CHECK(u.encode(*ut, cmd).str() == "18FF08D0#F0FFFFFFFFFFFFFF", "usv722 trim idle F0");
    CHECK(u.hasTrimIdle(*ut), "usv722 has an idle frame");
  }

  // Pulse timing (usv722: up 1.0 s, down 0.5 s)
  DriveCore core;
  core.configure(u);
  core.setOwner(OWNER_MANUAL);
  double t = 100;
  core.trimCommand(1, t);
  DriveCommand cmd = core.update(t);
  CHECK(cmd.trim == TRIM_UP, "trim up starts at once");
  cmd = core.update(t + 0.95);
  CHECK(cmd.trim == TRIM_UP, "trim up still running at 0.95 s");
  cmd = core.update(t + 1.0);
  CHECK(cmd.trim == TRIM_IDLE, "trim up stops by itself at 1.0 s");

  t = 200;
  core.trimCommand(-1, t);
  cmd = core.update(t + 0.45);
  CHECK(cmd.trim == TRIM_DOWN, "trim down running at 0.45 s");
  cmd = core.update(t + 0.5);
  CHECK(cmd.trim == TRIM_IDLE, "trim down stops at 0.5 s (shorter pulse)");

  // A repeated command restarts the pulse; it does not add up
  t = 300;
  core.trimCommand(1, t);
  core.update(t + 0.5);
  core.trimCommand(1, t + 0.5);
  cmd = core.update(t + 1.45);
  CHECK(cmd.trim == TRIM_UP, "second press restarts the pulse");
  cmd = core.update(t + 1.5);
  CHECK(cmd.trim == TRIM_IDLE, "restarted pulse is still only 1.0 s long");

  // Stop, reversal, lost link, owner change, reset
  core.trimCommand(1, 400);
  core.trimCommand(0, 400.2);
  CHECK(core.update(400.2).trim == TRIM_IDLE, "TRIM=0 stops at once");
  core.trimCommand(1, 401);
  core.trimCommand(-1, 401.2);
  CHECK(core.update(401.3).trim == TRIM_DOWN, "down replaces up at once");
  core.trimCommand(1, 402);
  core.setLinkOk(false);
  CHECK(core.update(402.1).trim == TRIM_IDLE, "lost link stops trim");
  core.setLinkOk(true);
  core.trimCommand(1, 403);
  core.setOwner(OWNER_AUTONOMY);
  CHECK(core.update(403.1).trim == TRIM_IDLE, "owner change stops trim");
  core.trimCommand(-1, 404);
  core.resetToNeutral(404.1);
  CHECK(core.update(404.1).trim == TRIM_IDLE, "reset stops trim");

  // Validation
  string base = "profile = t\ncan_interface = vcan0\ncapabilities = trim\n"
                "tx_msg = name=trim, cap=trim, id=18FF08D0, dlc=8, period_ms=100\n";
  string field = "tx_field = msg=trim, src=trim, byte=0, bits=8, map=enum, values=up:F1|down:F4\n";
  Profile ok;
  ok.loadFromString(base + field, g_profile_dir);
  CHECK(ok.valid(), "trim-only profile is valid");
  printErrors(ok);

  Profile p1;
  p1.loadFromString(base + field + "trim_up_secs = 6\n", g_profile_dir);
  CHECK(!p1.valid(), "trim pulse longer than 5 s is rejected");
  Profile p2;
  p2.loadFromString(base + "tx_field = msg=trim, src=trim, byte=0, bits=8, map=enum, values=up:F1\n",
                    g_profile_dir);
  CHECK(!p2.valid(), "trim enum without down is rejected");
  Profile p3;
  p3.loadFromString("profile = t\ncan_interface = vcan0\ncapabilities = steer\n"
                    "tx_msg = name=steer, cap=steer, id=18FF11D0, dlc=8, period_ms=100\n"
                    "tx_field = msg=steer, src=trim, byte=0, bits=8, map=enum, values=up:F1|down:F4\n",
                    g_profile_dir);
  CHECK(!p3.valid(), "trim field in a steer message is rejected");
}

//------------------------------------------------------------------
// Manual gear: gear from DESIRED_GEAR, thrust is throttle only

static void testManualGear()
{
  Profile u;
  loadUsv722NoRudderShaping(u);   // neutral 1.0 s, settle 0.5 s
  DriveCore core;
  core.configure(u);
  core.setManualGearFromCommand(true);
  core.setOwner(OWNER_MANUAL);
  CHECK(core.gearFromCommand(), "manual + command mode uses the gear command");

  double t = 10;
  DriveCommand cmd;

  // Thrust in neutral does nothing
  core.setThrust(30);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_NEUTRAL && cmd.thrust_pct == 0, "thrust in neutral: no gear, no throttle");

  // Forward with 0 thrust: in gear at idle
  core.setGear(GEAR_FWD);
  CHECK(core.thrustSetpoint() == 0, "selecting a gear resets thrust to 0");
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_FWD && cmd.thrust_pct == 0, "forward engaged at idle (0 thrust)");
  for(int i=0; i<100; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_FWD && cmd.thrust_pct == 0, "stays in forward at idle");

  // Throttle in forward; a negative value is treated as throttle only
  core.setThrust(30);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_FWD && cmd.thrust_pct == 30, "forward 30%");
  core.setThrust(-20);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_FWD && cmd.thrust_pct == 20, "sign ignored in manual gear mode");

  // Forward -> reverse: throttle reset, neutral pause, then reverse at idle
  core.setGear(GEAR_REV);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_NEUTRAL && cmd.thrust_pct == 0, "shift goes through neutral");
  double t0 = t;
  while(t < t0 + 1.0)
    cmd = core.update(t += 0.05);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_REV && cmd.thrust_pct == 0, "reverse engaged at idle after the pause");
  for(int i=0; i<20; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.thrust_pct == 0, "old throttle is not re-applied after the shift");
  core.setThrust(25);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_REV && cmd.thrust_pct == -25, "reverse 25% once throttle is applied again");

  // Same gear again does not reset throttle
  core.setGear(GEAR_REV);
  CHECK(core.thrustSetpoint() == 25, "re-selecting the same gear keeps throttle");

  // Neutral cuts throttle
  core.setGear(GEAR_NEUTRAL);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_NEUTRAL && cmd.thrust_pct == 0, "neutral button: neutral, 0 throttle");

  // Lost link: neutral, 0, and stays there until a new gear command
  core.setGear(GEAR_FWD);
  core.setThrust(40);
  for(int i=0; i<30; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_FWD && cmd.thrust_pct == 40, "forward 40% before link loss");
  core.setLinkOk(false);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_NEUTRAL && cmd.thrust_pct == 0, "lost link: neutral, 0");
  core.setLinkOk(true);
  for(int i=0; i<40; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_NEUTRAL && core.gearSetpoint() == GEAR_NEUTRAL,
        "stays in neutral after the link returns");

  // Autonomy ignores the gear command and uses the thrust sign
  core.setOwner(OWNER_AUTONOMY);
  CHECK(!core.gearFromCommand(), "autonomy does not use the gear command");
  core.setGear(GEAR_REV);
  core.setThrust(40);
  for(int i=0; i<40; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_FWD && cmd.thrust_pct == 40, "autonomy: +40 thrust -> forward");
}

//------------------------------------------------------------------
// Manual control, one lever (manual_gear = thrust_sign)

static void testManualOneLever()
{
  Profile u;
  loadUsv722NoRudderShaping(u);   // neutral 1.0 s, settle 0.5 s
  DriveCore core;
  core.configure(u);
  core.setManualGearFromCommand(false);
  core.setOwner(OWNER_MANUAL);
  CHECK(!core.gearFromCommand(), "one lever: gear comes from the thrust sign");

  double t = 10;
  DriveCommand cmd;

  // +30: forward engages at once, throttle after the settle time
  core.setThrust(30);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_FWD && cmd.thrust_pct == 0, "+30: forward, throttle held while settling");
  for(int i=0; i<12; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_FWD && cmd.thrust_pct == 30, "+30: forward 30% after settling");

  // -20: through neutral for 1.0 s, then reverse, then throttle
  core.setThrust(-20);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_NEUTRAL && cmd.thrust_pct == 0, "-20: neutral first");
  for(int i=0; i<10; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_NEUTRAL, "-20: still neutral before 1.0 s");
  for(int i=0; i<30; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_REV && cmd.thrust_pct == -20, "-20: reverse 20% after neutral + settle");

  // 0: neutral
  core.setThrust(0);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_NEUTRAL && cmd.thrust_pct == 0, "0: neutral");

  // Lost link: neutral and 0 until a new command
  core.setThrust(40);
  for(int i=0; i<40; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_FWD && cmd.thrust_pct == 40, "forward 40% before link loss");
  core.setLinkOk(false);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_NEUTRAL && cmd.thrust_pct == 0, "lost link: neutral, 0");
  core.setLinkOk(true);
  for(int i=0; i<40; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_NEUTRAL && cmd.thrust_pct == 0, "stays neutral after the link returns");
}

//------------------------------------------------------------------
// AutoTuning: planchaPID settings from the boat profile

static void testAutoTuning()
{
  // Built-in defaults = the values that used to be hardcoded in planchaPID
  AutoTuning d;
  CHECK(fabs(d.feedForward(5) - 14) < 1e-9, "default ff 5 -> 14 %");
  CHECK(fabs(d.feedForward(9) - 31) < 1e-9, "default ff 9 -> 31 %");
  CHECK(fabs(d.feedForward(50) - 44) < 1e-9, "default ff held flat above 20");
  double kp, ki;
  d.gains(1, kp, ki);   CHECK(kp == 0.35 && ki == 0.005, "default gains below 2");
  d.gains(2, kp, ki);   CHECK(kp == 0.25 && ki == 0.004, "default gains at 2");
  d.gains(7, kp, ki);   CHECK(kp == 0.16 && ki == 0.003, "default gains 6..12");
  d.gains(30, kp, ki);  CHECK(kp == 0.10 && ki == 0.002, "default gains above 12");
  CHECK(fabs(d.rudderLimit(10) - (20 + 5.0 / 13 * (8 - 20))) < 1e-9, "default rudder limit at 10");
  CHECK(d.thrust_max == 50 && d.i_authority == 2, "default thrust max 50, I-term 2");

  // The Mercury profile spells out the same values: no behaviour change
  Profile m;
  CHECK(m.load(g_profile_dir + "/mercury_sterndrive.prf"), "mercury loads with auto_* keys");
  AutoTuning mt;
  CHECK(mt.configure(m), "mercury tuning configures");
  for(double v=0; v<=25; v+=0.5) {
    double a, b, c, e;
    d.gains(v, a, b);
    mt.gains(v, c, e);
    CHECK(fabs(mt.feedForward(v) - d.feedForward(v)) < 1e-9 && a == c && b == e &&
          fabs(mt.rudderLimit(v) - d.rudderLimit(v)) < 1e-9, "mercury = old tuning at " << v);
  }

  // USV-722: measured curve, wider rudder limits
  Profile u;
  CHECK(u.load(g_profile_dir + "/usv722_glendinning.prf"), "usv722 loads with auto_* keys");
  printErrors(u);
  AutoTuning ut;
  CHECK(ut.configure(u), "usv722 tuning configures");
  CHECK(fabs(ut.feedForward(5) - 40) < 1e-9, "usv722 ff 5 kn -> lever 40 %");
  CHECK(ut.rudderLimit(2) == 40 && ut.rudderLimit(20) == 10, "usv722 rudder 40 % slow, 10 % planing");
  CHECK(ut.thrust_max == 100 && ut.i_authority == 10, "usv722 thrust max 100, I-term 10");

  // Validation
  CHECK(rejects("auto_speed_ff = 0:0|5:20|4:30\n", "speeds must increase"), "ff speeds must increase");
  CHECK(rejects("auto_speed_ff = 0:0|5:120\n", "percent must be 0..100"), "ff thrust above 100");
  CHECK(rejects("auto_speed_gains = 2:0.3\n", "up_to:kp:ki"), "gain step needs three values");
  CHECK(rejects("auto_rudder_min_pct = 50\n", "above auto_rudder_max_pct"), "rudder min above max");
  CHECK(rejects("auto_thrust_max_pct = 150\n", "must be 0..100"), "thrust max above 100");
}

//------------------------------------------------------------------
// Speed-dependent rudder limit (rudder_limit_speed), manual and autonomy

static void testRudderSpeedLimit()
{
  Profile p;
  p.loadFromString(
    "profile = t\ncan_interface = vcan0\ncapabilities = steer\n"
    "tx_msg = name=steer, cap=steer, id=18FF11D0, dlc=8, period_ms=100\n"
    "tx_field = msg=steer, src=rudder_pct, byte=0, bits=16, map=points, points=-100:0|0:511|100:1012\n"
    "rudder_limit_speed = 0:100|5:100|10:40|18:12|27:8\n", g_profile_dir);
  CHECK(p.valid(), "profile with rudder_limit_speed is valid");
  printErrors(p);

  DriveCore core;
  core.configure(p);
  CHECK(core.speedLimitActive(), "speed limit active");
  core.setOwner(OWNER_MANUAL);
  core.setRudder(100);

  double t = 100;
  core.setSpeed(2, t);
  CHECK(core.update(t += 0.05).rudder_pct == 100, "2 kn: full rudder allowed");
  core.setSpeed(10, t);
  CHECK(core.update(t += 0.05).rudder_pct == 40, "10 kn: rudder 40 %");
  core.setSpeed(14, t);
  CHECK(fabs(core.update(t += 0.05).rudder_pct - 26) < 1e-9, "14 kn: rudder 26 % (interpolated)");
  core.setSpeed(27, t);
  CHECK(core.update(t += 0.05).rudder_pct == 8, "27 kn: rudder 8 %");
  core.setRudder(-100);
  CHECK(core.update(t += 0.05).rudder_pct == -8, "27 kn: -8 % to port as well");
  core.setSpeed(40, t);
  CHECK(core.update(t += 0.05).rudder_pct == -8, "above the table: last value");

  // Stale speed: most cautious value, and recovers when NAV_SPEED returns
  CHECK(core.update(t += 5).rudder_pct == -8 && core.speedStale(), "no NAV_SPEED for 5 s: 8 %");
  core.setSpeed(2, t);
  CHECK(core.update(t += 0.05).rudder_pct == -100 && !core.speedStale(), "NAV_SPEED back: full rudder");

  // Autonomy goes through the same limit
  core.setOwner(OWNER_AUTONOMY);
  core.setRudder(60);
  core.setSpeed(18, t);
  CHECK(core.update(t += 0.05).rudder_pct == 12, "autonomy at 18 kn: 12 %");

  // Without the key nothing changes
  Profile m;
  m.load(g_profile_dir + "/mercury_sterndrive.prf");
  DriveCore mc;
  mc.configure(m);
  mc.setRudder(100);
  CHECK(!mc.speedLimitActive() && mc.update(1.0).rudder_pct == 100, "no rudder_limit_speed: no speed limit");

  // USV-722 profile has the table and a slew limit
  Profile u;
  CHECK(u.load(g_profile_dir + "/usv722_glendinning.prf"), "usv722 loads with rudder_limit_speed");
  CHECK(u.setting("rudder_rate_pct_s") == 50, "usv722 rudder slew 50 %/s");

  CHECK(rejects("rudder_limit_speed = 0:100|10:150\n", "percent must be 0..100"), "rudder limit above 100");
  CHECK(rejects("rudder_limit_speed = 10:100|5:50\n", "speeds must increase"), "rudder limit speeds must increase");
}

//------------------------------------------------------------------
// Throttle ramp-up (thrust_rate_up_pct_s): gradual increase, instant cut

static void testThrottleRampUp()
{
  Profile p;
  p.loadFromString(
    "profile = t\ncan_interface = vcan0\ncapabilities = thrust\n"
    "tx_msg = name=throttle, cap=thrust, id=0CF003D0, dlc=8, period_ms=100\n"
    "tx_field = msg=throttle, src=thrust_abs_pct, byte=1, bits=8, map=linear, scale=2.5\n"
    "thrust_rate_up_pct_s = 10\n", g_profile_dir);
  CHECK(p.valid(), "profile with thrust_rate_up_pct_s is valid");
  printErrors(p);

  DriveCore core;
  core.configure(p);
  core.setOwner(OWNER_AUTONOMY);
  double t = 0;
  DriveCommand cmd = core.update(t);

  // 0 -> 95 %: rises 10 %/s, never jumps
  core.setThrust(95);
  double prev = 0;
  bool smooth = true;
  for(int i=0; i<20; i++) {               // 1 s
    cmd = core.update(t += 0.05);
    if(cmd.thrust_pct - prev > 0.5 + 1e-9) smooth = false;
    prev = cmd.thrust_pct;
  }
  CHECK(smooth, "throttle rises at most 0.5 % per 50 ms tick");
  CHECK(fabs(cmd.thrust_pct - 10) < 1e-6, "after 1 s: 10 %, got " << cmd.thrust_pct);
  for(int i=0; i<200; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.thrust_pct == 95, "after 11 s: 95 %");

  // A cut is immediate
  core.setThrust(20);
  cmd = core.update(t += 0.05);
  CHECK(cmd.thrust_pct == 20, "95 -> 20 % at once");
  core.setThrust(0);
  cmd = core.update(t += 0.05);
  CHECK(cmd.thrust_pct == 0 && cmd.gear == GEAR_NEUTRAL, "stop: 0 and neutral at once");

  // Reverse ramps from 0 as well
  core.setThrust(-50);
  cmd = core.update(t += 0.05);
  CHECK(cmd.gear == GEAR_REV && cmd.thrust_pct > -1, "reverse starts from 0, got " << cmd.thrust_pct);
  for(int i=0; i<20; i++)
    cmd = core.update(t += 0.05);
  CHECK(fabs(cmd.thrust_pct + 10.5) < 1e-6, "reverse after ~1 s: -10.5 %, got " << cmd.thrust_pct);

  // Lost link in manual: 0 at once
  core.setOwner(OWNER_MANUAL);
  core.setThrust(60);
  for(int i=0; i<200; i++)
    cmd = core.update(t += 0.05);
  CHECK(cmd.thrust_pct == 60, "manual ramps up to 60 %");
  core.setLinkOk(false);
  cmd = core.update(t += 0.05);
  CHECK(cmd.thrust_pct == 0, "lost link: 0 at once");

  // Both boat profiles ramp up
  Profile u, m;
  u.load(g_profile_dir + "/usv722_glendinning.prf");
  m.load(g_profile_dir + "/mercury_sterndrive.prf");
  CHECK(u.setting("thrust_rate_up_pct_s") > 0 && m.setting("thrust_rate_up_pct_s") > 0,
        "both boats have a throttle ramp-up");
  CHECK(rejects("thrust_rate_up_pct_s = -1\n", "must not be negative"), "negative ramp-up rejected");
}

//------------------------------------------------------------------

int main(int argc, char* argv[])
{
  g_profile_dir = (argc > 1) ? argv[1] : "profiles";

  cout << "lib_canprofile tests (profiles from " << g_profile_dir << ")" << endl;
  testMercuryFrames();
  testTableMap();
  testUsv722Frames();
  testUsv722Feedback();
  testValidation();
  testDriveCore();
  testTrim();
  testManualGear();
  testManualOneLever();
  testAutoTuning();
  testRudderSpeedLimit();
  testThrottleRampUp();

  cout << g_checks << " checks, " << g_failures << " failed" << endl;
  return g_failures ? 1 : 0;
}
