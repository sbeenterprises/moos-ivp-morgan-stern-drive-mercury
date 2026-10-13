/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: CAN.cpp                                        */
/*    DATE: December 29th, 1963                             */
/*          October 2026: rewritten around lib_canprofile.  */
/*          No CAN IDs or payloads live in this file; they  */
/*          come from the boat's profile (profiles/NAME.prf).  */
/************************************************************/

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include "MBUtils.h"
#include "ACTable.h"
#include "CAN.h"

using namespace std;
using namespace canprofile;

//---------------------------------------------------------
// Constructor()

CAN::CAN()
{
  m_link_stale_secs   = 3.0;
  m_manual_sources.insert("iWebSocket");
  m_autonomy_sources.insert("planchaPID");
  m_autonomy_sources.insert("pHelmIvP");     // end-of-mission zeros (alpha.bhv endflags)
  m_autonomy_sources.insert("iWebSocket");   // AUTONOMOUS_STOP / TRAIL_FOLLOW=off zeros
  m_trim_sources.insert("iWebSocket");       // trim is an operator function only
  m_core.setManualGearFromCommand(true);     // manual: gear from DESIRED_GEAR

  m_profile_ok        = false;
  m_can_socket        = -1;
  m_last_init_attempt = -1e9;
  m_start_time        = 0;
  m_can_send          = false;   // nothing is sent until CAN_SEND=true
  m_ui_link_ok        = false;
  m_ui_link_time      = -1e9;
  m_tx_errors         = 0;
  m_rx_frames         = 0;
  m_last_iterate      = 0;
  m_tick              = 0.05;
  m_rejected          = 0;
  m_mismatch_since    = -1;
  m_mismatch          = false;
  m_last_status_post  = 0;
}

//---------------------------------------------------------
// Destructor

CAN::~CAN()
{
  closeCAN();
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool CAN::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    if(key == "DESIRED_RUDDER")
      handleDemand(msg, true);
    else if(key == "DESIRED_THRUST")
      handleDemand(msg, false);
    else if(key == "TRIM")
      handleTrim(msg);
    else if(key == "DESIRED_GEAR")
      handleGear(msg);
    else if(key == "MOOS_MANUAL_OVERRIDE") {
      bool manual = (tolower(stripBlankEnds(msg.GetString())) == "true");
      m_core.setOwner(manual ? OWNER_MANUAL : OWNER_AUTONOMY);
    }
    else if(key == "UI_LINK_OK") {
      string v = msg.IsString() ? tolower(stripBlankEnds(msg.GetString())) : "";
      m_ui_link_ok   = msg.IsString() ? (v == "true") : (msg.GetDouble() != 0);
      m_ui_link_time = MOOSTime();
    }
    else if(key == "CAN_SEND") {
      string v = msg.IsString() ? tolower(stripBlankEnds(msg.GetString())) : "";
      bool enable = msg.IsString() ? (v == "true") : (msg.GetDouble() != 0);
      double now = MOOSTime();
      if(enable && !m_can_send)
        m_core.resetToNeutral(now);   // never resume an old command
      else if(!enable && m_can_send)
        sendNeutralOnce(now);         // leave the drive in neutral, then go quiet
      m_can_send = enable;
    }
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: handleDemand()
//   Accept a rudder/thrust demand only from a process that belongs
//   to the current owner. In MANUAL that is SS UI (iWebSocket) only,
//   so planchaPID or a mission end-flag can no longer overwrite the
//   operator's setting.

void CAN::handleDemand(const CMOOSMsg& msg, bool is_rudder)
{
  string src = msg.GetSource();
  const set<string>& allowed = (m_core.owner() == OWNER_MANUAL) ?
    m_manual_sources : m_autonomy_sources;

  if(allowed.count(src) == 0) {
    m_rejected++;
    m_last_rejected = msg.GetKey() + " from " + src + " (owner " +
      ownerName(m_core.owner()) + ")";
    return;
  }

  double val = msg.IsDouble() ? msg.GetDouble() : atof(msg.GetString().c_str());
  if(is_rudder)
    m_core.setRudder(val);
  else
    m_core.setThrust(val);
}

//---------------------------------------------------------
// Procedure: handleGear()
//   DESIRED_GEAR from SS UI in manual control. iWebSocket posts A / N / R
//   (forward / neutral / reverse); F, FWD, FORWARD, 1, REV, REVERSE, -1,
//   NEUTRAL and 0 are accepted too. Only used while the owner is MANUAL
//   and manual_gear = command; otherwise the gear follows the thrust sign.

void CAN::handleGear(const CMOOSMsg& msg)
{
  if(m_core.owner() != OWNER_MANUAL || !m_core.manualGearFromCommand())
    return;
  if(m_manual_sources.count(msg.GetSource()) == 0) {
    m_rejected++;
    m_last_rejected = "DESIRED_GEAR from " + msg.GetSource();
    return;
  }

  string v;
  if(msg.IsDouble())
    v = (msg.GetDouble() > 0.5) ? "1" : ((msg.GetDouble() < -0.5) ? "-1" : "0");
  else
    v = toupper(stripBlankEnds(msg.GetString()));

  if(v == "A" || v == "F" || v == "FWD" || v == "FORWARD" || v == "1")
    m_core.setGear(GEAR_FWD);
  else if(v == "R" || v == "REV" || v == "REVERSE" || v == "-1")
    m_core.setGear(GEAR_REV);
  else if(v == "N" || v == "NEUTRAL" || v == "0")
    m_core.setGear(GEAR_NEUTRAL);
  else
    reportRunWarning("DESIRED_GEAR ignored, unknown value: " + msg.GetString());
}

//---------------------------------------------------------
// Procedure: handleTrim()
//   TRIM = 1 (up), -1 (down), 0 (stop); also accepts up/down/stop.
//   Each command is one pulse of trim_up_secs / trim_down_secs from the
//   profile. Only accepted from trim_sources (SS UI) and only while CAN
//   sending is on and the profile has the trim capability.

void CAN::handleTrim(const CMOOSMsg& msg)
{
  double now = MOOSTime();
  int dir = 0;
  if(msg.IsDouble())
    dir = (msg.GetDouble() > 0.5) ? 1 : ((msg.GetDouble() < -0.5) ? -1 : 0);
  else {
    string v = tolower(stripBlankEnds(msg.GetString()));
    if(v == "up" || v == "1")
      dir = 1;
    else if(v == "down" || v == "-1")
      dir = -1;
  }

  // A stop is always honoured, whoever sends it
  if(dir == 0) {
    m_core.stopTrim();
    return;
  }

  if(m_trim_sources.count(msg.GetSource()) == 0) {
    m_rejected++;
    m_last_rejected = "TRIM from " + msg.GetSource();
    return;
  }
  if(!m_profile_ok || !m_profile.hasCapability("trim")) {
    m_trim_note = "TRIM ignored: this boat's profile has no trim capability";
    reportEvent(m_trim_note);
    return;
  }
  if(!m_can_send) {
    m_trim_note = "TRIM ignored: CAN_SEND is false";
    reportEvent(m_trim_note);
    return;
  }
  m_trim_note = "";
  m_core.trimCommand(dir, now);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool CAN::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool CAN::Iterate()
{
  AppCastingMOOSApp::Iterate();
  double now = MOOSTime();
  if(m_last_iterate > 0 && now > m_last_iterate)
    m_tick = 0.9 * m_tick + 0.1 * (now - m_last_iterate);
  m_last_iterate = now;

  // Keep trying to open the bus (slcan0 may come up after iCAN)
  if(m_profile_ok && m_can_socket == -1 && now - m_last_init_attempt > 5.0) {
    m_last_init_attempt = now;
    if(initCAN())
      m_start_time = now;   // listen guard_secs before the first frame
  }

  readFrames(now);

  bool link_ok = m_ui_link_ok && (now - m_ui_link_time <= m_link_stale_secs);
  m_core.setLinkOk(link_ok);
  DriveCommand cmd = m_core.update(now);

  if(m_profile_ok && m_can_socket != -1 && m_can_send) {
    if(m_last_cmd.trim != TRIM_IDLE && cmd.trim == TRIM_IDLE)
      sendTrimIdle(now);   // pulse just ended: tell the motor to stop
    sendDue(cmd, now);
  }

  checkRudderFeedback(cmd, now);
  postStatus(cmd, now);
  m_last_cmd = cmd;

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool CAN::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "profile_file") {
      m_profile_file = value;
      handled = true;
    }
    else if(param == "can_interface") {
      m_can_interface = value;
      handled = true;
    }
    else if(param == "manual_sources" || param == "autonomy_sources" ||
            param == "trim_sources") {
      set<string>& dest = (param == "manual_sources") ? m_manual_sources :
        ((param == "trim_sources") ? m_trim_sources : m_autonomy_sources);
      dest.clear();
      vector<string> names = parseString(value, ',');
      for(unsigned int i=0; i<names.size(); i++)
        if(stripBlankEnds(names[i]) != "")
          dest.insert(stripBlankEnds(names[i]));
      handled = true;
    }
    else if(param == "manual_gear") {
      string v = tolower(value);
      if(v == "command" || v == "thrust_sign") {
        m_core.setManualGearFromCommand(v == "command");
        handled = true;
      }
    }
    else if(param == "link_stale_secs" && isNumber(value)) {
      m_link_stale_secs = atof(value.c_str());
      handled = (m_link_stale_secs > 0);
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  // Load and validate the boat profile. Any error leaves iCAN running
  // but inhibited: it never transmits, and the errors are on the appcast.
  if(m_profile_file == "")
    reportConfigWarning("profile_file is not set; iCAN will not transmit");
  else if(!m_profile.load(m_profile_file)) {
    reportConfigWarning("Profile " + m_profile_file + " is invalid; iCAN will not transmit");
    for(size_t i=0; i<m_profile.errors().size(); i++)
      reportConfigWarning("  " + m_profile.errors()[i]);
  }
  else {
    m_profile_ok = true;
    m_core.configure(m_profile);
    if(m_can_interface == "")
      m_can_interface = m_profile.canInterface();
  }

  // Sized even for an invalid profile, so the appcast can list its messages
  size_t n = m_profile.txMessages().size();
  m_last_sent.assign(n, -1e9);
  m_foreign_seen.assign(n, -1e9);
  m_sent_count.assign(n, 0);

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void CAN::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("DESIRED_RUDDER", 0);
  Register("DESIRED_THRUST", 0);
  Register("MOOS_MANUAL_OVERRIDE", 0);
  Register("UI_LINK_OK", 0);
  Register("CAN_SEND", 0);
  Register("TRIM", 0);
  Register("DESIRED_GEAR", 0);
}

//---------------------------------------------------------
// Procedure: initCAN()

bool CAN::initCAN()
{
  closeCAN();
  m_can_error = "";

  int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if(s < 0) {
    m_can_error = string("socket(PF_CAN): ") + strerror(errno);
    return(false);
  }

  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, m_can_interface.c_str(), IFNAMSIZ - 1);
  if(ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
    m_can_error = "interface " + m_can_interface + " not found: " + strerror(errno);
    close(s);
    return(false);
  }

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    m_can_error = string("bind: ") + strerror(errno);
    close(s);
    return(false);
  }

  // Non-blocking reads; our own transmissions are not echoed back to
  // this socket (CAN_RAW_RECV_OWN_MSGS is off by default), so any
  // frame we receive on one of our IDs came from another node.
  fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
  m_can_socket = s;
  return(true);
}

void CAN::closeCAN()
{
  if(m_can_socket != -1)
    close(m_can_socket);
  m_can_socket = -1;
}

//---------------------------------------------------------
// Procedure: readFrames()
//   Drain the socket: note other senders on our IDs and decode
//   the profile's feedback fields.

void CAN::readFrames(double now)
{
  if(m_can_socket == -1)
    return;

  for(int n=0; n<500; n++) {
    struct can_frame fr;
    ssize_t got = read(m_can_socket, &fr, sizeof(fr));
    if(got < 0) {
      if(errno != EAGAIN && errno != EWOULDBLOCK) {
        m_can_error = string("read: ") + strerror(errno);
        closeCAN();   // reopened by Iterate
      }
      return;
    }
    if(got < (ssize_t)sizeof(fr) || (fr.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG)))
      continue;

    m_rx_frames++;
    bool ext = (fr.can_id & CAN_EFF_FLAG) != 0;
    uint32_t id = ext ? (fr.can_id & CAN_EFF_MASK) : (fr.can_id & CAN_SFF_MASK);

    const vector<TxMsg>& tx = m_profile.txMessages();
    for(size_t i=0; i<tx.size(); i++)
      if(tx[i].id == id && tx[i].ext == ext)
        m_foreign_seen[i] = now;

    vector<RxValue> vals = m_profile.decode(id, ext, fr.data, fr.can_dlc);
    for(size_t i=0; i<vals.size(); i++)
      postFeedback(vals[i], now);
  }
}

//---------------------------------------------------------
// Procedure: postFeedback()
//   Post on change, and at least once a second.

void CAN::postFeedback(const RxValue& v, double now)
{
  m_fb_time[v.name] = now;
  bool changed;
  if(v.is_string) {
    changed = (m_fb_sval[v.name] != v.sval);
    m_fb_sval[v.name] = v.sval;
  }
  else {
    changed = (m_fb_dval.count(v.name) == 0 || m_fb_dval[v.name] != v.dval);
    m_fb_dval[v.name] = v.dval;
  }
  if(changed || now - m_fb_posted[v.name] >= 1.0) {
    if(v.is_string)
      Notify(v.name, v.sval);
    else
      Notify(v.name, v.dval);
    m_fb_posted[v.name] = now;
  }
}

//---------------------------------------------------------
// Procedure: msgEnabled()

bool CAN::msgEnabled(const TxMsg& msg) const
{
  return m_profile.hasCapability(msg.cap);
}

//---------------------------------------------------------
// Procedure: msgState()
//   What is happening with one tx message right now

string CAN::msgState(size_t ix, double now) const
{
  const TxMsg& msg = m_profile.txMessages()[ix];
  if(!msgEnabled(msg))
    return "off (capability not enabled)";
  if(!m_profile_ok)
    return "inhibited";
  if(m_can_socket == -1)
    return "no bus";
  double guard = m_profile.setting("guard_secs");
  if(msg.guard && now - m_foreign_seen[ix] < guard)
    return "BLOCKED: another node sends this ID";
  if(msg.guard && now - m_start_time < guard)
    return "listening";
  if(!m_can_send)
    return "standby (CAN_SEND=false)";
  if(msg.cap == "trim" && m_core.currentTrim() == TRIM_IDLE)
    return "idle (sent only during a trim pulse)";
  return "sending";
}

//---------------------------------------------------------
// Procedure: sendDue()

void CAN::sendDue(const DriveCommand& cmd, double now)
{
  const vector<TxMsg>& tx = m_profile.txMessages();
  double guard = m_profile.setting("guard_secs");
  for(size_t i=0; i<tx.size(); i++) {
    const TxMsg& msg = tx[i];
    if(!msgEnabled(msg))
      continue;
    if(msg.cap == "trim" && cmd.trim == TRIM_IDLE)
      continue;            // trim frames only during a pulse
    if(msg.guard && (now - m_foreign_seen[i] < guard || now - m_start_time < guard))
      continue;
    // Send when the period has elapsed, allowing half a tick of slack
    if(now - m_last_sent[i] + 0.5 * m_tick < msg.period_ms / 1000.0)
      continue;
    if(sendFrame(m_profile.encode(msg, cmd))) {
      m_last_sent[i] = now;
      m_sent_count[i]++;
    }
  }
}

//---------------------------------------------------------
// Procedure: sendNeutralOnce()
//   Called when CAN_SEND goes false: one frame of every enabled
//   message with rudder 0, throttle 0 and neutral gear.

void CAN::sendNeutralOnce(double now)
{
  if(!m_profile_ok || m_can_socket == -1)
    return;
  DriveCommand neutral;
  const vector<TxMsg>& tx = m_profile.txMessages();
  double guard = m_profile.setting("guard_secs");
  for(size_t i=0; i<tx.size(); i++) {
    if(!msgEnabled(tx[i]))
      continue;
    if(tx[i].cap == "trim" && !m_profile.hasTrimIdle(tx[i]))
      continue;            // no stop value defined: just stop sending
    if(tx[i].guard && now - m_foreign_seen[i] < guard)
      continue;
    sendFrame(m_profile.encode(tx[i], neutral));
  }
  m_core.resetToNeutral(now);
}

//---------------------------------------------------------
// Procedure: sendTrimIdle()
//   One idle frame on each trim message that defines one.

void CAN::sendTrimIdle(double now)
{
  DriveCommand idle = m_last_cmd;
  idle.trim = TRIM_IDLE;
  const vector<TxMsg>& tx = m_profile.txMessages();
  double guard = m_profile.setting("guard_secs");
  for(size_t i=0; i<tx.size(); i++) {
    if(tx[i].cap != "trim" || !msgEnabled(tx[i]) || !m_profile.hasTrimIdle(tx[i]))
      continue;
    if(tx[i].guard && now - m_foreign_seen[i] < guard)
      continue;
    if(sendFrame(m_profile.encode(tx[i], idle))) {
      m_last_sent[i] = now;
      m_sent_count[i]++;
    }
  }
}

bool CAN::sendFrame(const Frame& f)
{
  struct can_frame fr;
  memset(&fr, 0, sizeof(fr));
  fr.can_id  = f.ext ? (f.id | CAN_EFF_FLAG) : f.id;
  fr.can_dlc = f.dlc;
  memcpy(fr.data, f.data, 8);
  if(write(m_can_socket, &fr, sizeof(fr)) != (ssize_t)sizeof(fr)) {
    m_tx_errors++;
    m_can_error = string("write: ") + strerror(errno);
    return(false);
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: checkRudderFeedback()
//   Alarm when the actuator's reported position stays away from the
//   commanded rudder (for example the rudder drifting back to centre).

void CAN::checkRudderFeedback(const DriveCommand& cmd, double now)
{
  string fb = m_profile.stringSetting("rudder_feedback");
  double tol = m_profile.setting("rudder_alarm_pct");
  if(fb == "" || tol <= 0)
    return;

  bool steering = false;
  const vector<TxMsg>& tx = m_profile.txMessages();
  for(size_t i=0; i<tx.size(); i++)
    if(tx[i].cap == "steer" && msgState(i, now) == "sending")
      steering = true;

  bool fresh = m_fb_time.count(fb) && (now - m_fb_time[fb] < 1.0);
  bool off = steering && fresh && fabs(m_fb_dval[fb] - cmd.rudder_pct) > tol;

  if(!off)
    m_mismatch_since = -1;
  else if(m_mismatch_since < 0)
    m_mismatch_since = now;

  bool mismatch = off && (now - m_mismatch_since >= m_profile.setting("rudder_alarm_secs"));
  if(mismatch != m_mismatch) {
    m_mismatch = mismatch;
    Notify("CAN_RUDDER_MISMATCH", mismatch ? "true" : "false");
    if(mismatch)
      reportEvent("Rudder feedback " + doubleToString(m_fb_dval[fb], 1) +
                  "% differs from command " + doubleToString(cmd.rudder_pct, 1) + "%");
  }
}

//---------------------------------------------------------
// Procedure: postStatus()
//   On change, and once a second.

void CAN::postStatus(const DriveCommand& cmd, double now)
{
  string status;
  if(!m_profile_ok)
    status = "INHIBITED";
  else if(m_can_socket == -1)
    status = "NO_BUS";
  else if(!m_can_send)
    status = "STANDBY";
  else
    status = "ACTIVE";

  string conflict;
  const vector<TxMsg>& tx = m_profile.txMessages();
  for(size_t i=0; i<tx.size(); i++)
    if(msgState(i, now).find("BLOCKED") == 0)
      conflict += (conflict == "" ? "" : ",") + tx[i].name;

  bool changed = (status != m_last_status) || (conflict != m_last_conflict) ||
    (cmd.rudder_pct != m_last_cmd.rudder_pct) || (cmd.thrust_pct != m_last_cmd.thrust_pct) ||
    (cmd.gear != m_last_cmd.gear) || (cmd.trim != m_last_cmd.trim);
  if(!changed && now - m_last_status_post < 1.0)
    return;

  if(conflict != m_last_conflict && conflict != "")
    reportRunWarning("Not sending " + conflict + ": another node is transmitting the same ID");

  Notify("CAN_STATUS", status);
  Notify("CAN_CONTROL_OWNER", ownerName(m_core.owner()));
  Notify("CAN_LOST_LINK", m_core.lostLink() ? "true" : "false");
  Notify("CAN_CONFLICT", conflict == "" ? "none" : conflict);
  Notify("CAN_TX_RUDDER_PCT", cmd.rudder_pct);
  Notify("CAN_TX_THRUST_PCT", cmd.thrust_pct);
  Notify("CAN_TX_GEAR", gearName(cmd.gear));
  Notify("CAN_TRIM_STATE", trimName(cmd.trim));
  if(m_last_status_post == 0 || status != m_last_status) {
    Notify("CAN_PROFILE", m_profile_ok ? m_profile.name() : "none");
    Notify("CAN_PROFILE_HASH", m_profile.hashHex());
  }

  m_last_status = status;
  m_last_conflict = conflict;
  m_last_status_post = now;
}

//------------------------------------------------------------
// Procedure: buildReport()

bool CAN::buildReport()
{
  double now = MOOSTime();

  m_msgs << "============================================" << endl;
  m_msgs << "iCAN - drive commands from a boat profile   " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Profile:      " << (m_profile_file == "" ? "(not set)" : m_profile_file) << endl;
  m_msgs << "Boat:         " << (m_profile_ok ? m_profile.name() : "INVALID - not transmitting")
         << "   hash " << m_profile.hashHex() << endl;
  m_msgs << "Capabilities: " << m_profile.capabilitiesString() << endl;
  m_msgs << "Interface:    " << m_can_interface
         << (m_can_socket != -1 ? "  (open)" : "  (closed)")
         << (m_can_error == "" ? "" : "  " + m_can_error) << endl;
  m_msgs << "CAN_SEND:     " << (m_can_send ? "true" : "false") << endl;
  m_msgs << endl;

  if(!m_profile_ok) {
    for(size_t i=0; i<m_profile.errors().size(); i++)
      m_msgs << "  ERROR " << m_profile.errors()[i] << endl;
    m_msgs << endl;
  }

  m_msgs << "Owner:        " << ownerName(m_core.owner())
         << (m_core.lostLink() ? "   LOST LINK -> rudder/throttle set to lost-link values" : "")
         << endl;
  m_msgs << "UI link:      " << (m_ui_link_ok ? "ok" : "down")
         << "  (last UI_LINK_OK " << doubleToString(now - m_ui_link_time, 1) << " s ago)" << endl;
  m_msgs << "Setpoints:    rudder " << doubleToString(m_core.rudderSetpoint(), 1)
         << "%  thrust " << doubleToString(m_core.thrustSetpoint(), 1) << "%";
  if(m_core.gearFromCommand())
    m_msgs << "  gear " << gearName(m_core.gearSetpoint()) << " (DESIRED_GEAR)";
  else
    m_msgs << "  gear from thrust sign";
  m_msgs << endl;
  m_msgs << "Sending:      rudder " << doubleToString(m_last_cmd.rudder_pct, 1)
         << "%  thrust " << doubleToString(m_last_cmd.thrust_pct, 1)
         << "%  gear " << gearName(m_last_cmd.gear)
         << "  trim " << trimName(m_last_cmd.trim) << endl;
  if(m_trim_note != "")
    m_msgs << "Trim:         " << m_trim_note << endl;
  if(m_rejected)
    m_msgs << "Rejected:     " << m_rejected << "  last: " << m_last_rejected << endl;
  m_msgs << "Frames:       rx " << m_rx_frames << "  tx errors " << m_tx_errors << endl;
  m_msgs << endl;

  ACTable actab(5);
  actab << "Message | CAN ID | Period | Sent | State";
  actab.addHeaderLines();
  const vector<TxMsg>& tx = m_profile.txMessages();
  for(size_t i=0; i<tx.size(); i++) {
    char id[16];
    snprintf(id, sizeof(id), tx[i].ext ? "%08X" : "%03X", tx[i].id);
    actab << tx[i].name << id << (intToString(tx[i].period_ms) + " ms")
          << intToString((int)m_sent_count[i]) << msgState(i, now);
  }
  m_msgs << actab.getFormattedString() << endl;

  if(!m_fb_time.empty()) {
    m_msgs << endl;
    ACTable fbtab(3);
    fbtab << "Feedback | Value | Age";
    fbtab.addHeaderLines();
    map<string, double>::iterator p;
    for(p=m_fb_time.begin(); p!=m_fb_time.end(); p++) {
      string val = m_fb_sval.count(p->first) ? m_fb_sval[p->first]
                                             : doubleToString(m_fb_dval[p->first], 1);
      fbtab << p->first << val << (doubleToString(now - p->second, 1) + " s");
    }
    m_msgs << fbtab.getFormattedString();
  }

  return(true);
}
