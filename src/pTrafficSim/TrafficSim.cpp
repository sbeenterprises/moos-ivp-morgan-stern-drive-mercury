/************************************************************/
/*    NAME: Douglas Lima                                    */
/*    FILE: TrafficSim.cpp                                  */
/*    DATE: 2026                                            */
/************************************************************/

#include <iterator>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "MBUtils.h"
#include "ACTable.h"
#include "TrafficSim.h"
#include "AisEncode.h"

using namespace std;

static const double MPS_TO_KNOTS = 1.94384;
static const double DEG_TO_RAD   = M_PI / 180.0;
static const double MAX_TURN_RATE = 15.0;   // degrees/second, keeps loops plausible

//---------------------------------------------------------
// Contact constructor

TrafficSim::Contact::Contact()
{
  mmsi = 0;
  ais_shiptype = 70;          // "cargo, all ships of this type"
  node_type = "ship";
  x = 0; y = 0; hdg = 0; spd = 0;
  length = 30; beam = 8;
  navstatus = 0;              // under way using engine
  mode = MODE_TRANSIT;
  loop_idx = 0;
  next_pos_post = 0;
  next_static_post = 0;
}

//---------------------------------------------------------
// Constructor

TrafficSim::TrafficSim()
{
  m_enabled       = false;
  m_origin_lat    = 0;
  m_origin_lon    = 0;
  m_aivdm_host    = "127.0.0.1";
  m_aivdm_port    = 10112;
  m_mmsi_base     = 366999000;

  m_random_count    = 0;
  m_random_radius   = 1500;
  m_random_spd_min  = 2;
  m_random_spd_max  = 9;

  m_pos_interval    = 1.0;
  m_static_interval = 30.0;
  m_report_length   = 30.0;

  m_mmsi_next      = 0;
  m_ownship_x      = 0;
  m_ownship_y      = 0;
  m_ownship_known  = false;
  m_last_iterate   = 0;
  m_random_spawned = false;

  m_geodesy_ok = false;

  m_udp_sock = -1;
  m_udp_ok   = false;

  m_aivdm_sent        = 0;
  m_node_reports_sent = 0;
  m_seq_id            = 0;
}

//---------------------------------------------------------
// Destructor

TrafficSim::~TrafficSim()
{
  if(m_udp_sock >= 0)
    close(m_udp_sock);
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool TrafficSim::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if(key == "NAV_X") {
      m_ownship_x = msg.GetDouble();
      m_ownship_known = true;
    }
    else if(key == "NAV_Y") {
      m_ownship_y = msg.GetDouble();
      m_ownship_known = true;
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool TrafficSim::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool TrafficSim::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if(!m_enabled) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  double now = MOOSTime();
  double dt  = (m_last_iterate > 0) ? (now - m_last_iterate) : 0;
  m_last_iterate = now;

  // Random traffic is placed relative to the USV, so it waits for a first fix.
  if(!m_random_spawned && m_random_count > 0 && m_ownship_known) {
    spawnRandomContacts();
    m_random_spawned = true;
  }

  for(unsigned int i=0; i<m_contacts.size(); i++) {
    Contact& c = m_contacts[i];
    if(dt > 0 && dt < 5.0)      // ignore absurd steps, e.g. after a time warp change
      advanceContact(c, dt);
    postNodeReport(c);
    postAis(c, now);
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool TrafficSim::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  // Origin defaults to whatever the mission file declares, so a contact placed
  // at MOOS x/y lands on the same spot of the earth the rest of the community
  // thinks it is.
  m_MissionReader.GetValue("LatOrigin", m_origin_lat);
  m_MissionReader.GetValue("LongOrigin", m_origin_lon);

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;
    bool handled = false;

    if(param == "enable_simulation") {
      m_enabled = (tolower(value) == "true");
      handled = true;
    }
    else if(param == "origin_lat") {
      m_origin_lat = atof(value.c_str());
      handled = true;
    }
    else if(param == "origin_lon") {
      m_origin_lon = atof(value.c_str());
      handled = true;
    }
    else if(param == "aivdm_host") {
      m_aivdm_host = value;
      handled = true;
    }
    else if(param == "aivdm_port") {
      m_aivdm_port = atoi(value.c_str());
      handled = true;
    }
    else if(param == "mmsi_base") {
      m_mmsi_base = (unsigned int) atol(value.c_str());
      handled = true;
    }
    else if(param == "random_contacts") {
      m_random_count = (unsigned int) atoi(value.c_str());
      handled = true;
    }
    else if(param == "random_radius") {
      m_random_radius = atof(value.c_str());
      handled = true;
    }
    else if(param == "random_spd_min") {
      m_random_spd_min = atof(value.c_str());
      handled = true;
    }
    else if(param == "random_spd_max") {
      m_random_spd_max = atof(value.c_str());
      handled = true;
    }
    else if(param == "position_interval") {
      m_pos_interval = atof(value.c_str());
      handled = true;
    }
    else if(param == "static_interval") {
      m_static_interval = atof(value.c_str());
      handled = true;
    }
    else if(param == "report_length") {
      m_report_length = atof(value.c_str());
      handled = true;
    }
    else if(param == "contact")
      handled = handleConfigContact(value);
    else if(param == "loop")
      handled = handleConfigLoop(value);

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  m_mmsi_next = m_mmsi_base;
  // Contacts declared before mmsi_base was read still need their identity.
  for(unsigned int i=0; i<m_contacts.size(); i++) {
    if(m_contacts[i].mmsi == 0) {
      m_contacts[i].mmsi  = nextMmsi();
      m_contacts[i].vname = "AIS" + uintToString(m_contacts[i].mmsi);
      if(m_contacts[i].callsign == "")
        m_contacts[i].callsign = "SIM" + uintToString(m_contacts[i].mmsi % 10000);
    }
  }

  if(!m_enabled) {
    reportConfigWarning("pTrafficSim is INACTIVE: set enable_simulation=true to "
                        "generate simulated AIS traffic. It is deliberately off "
                        "by default so it can never run on a real vessel.");
  }

  if(m_geodesy.Initialise(m_origin_lat, m_origin_lon))
    m_geodesy_ok = true;
  else
    reportConfigWarning("Geodesy failed to initialise - no AIS will be sent.");

  if(m_enabled && !openAisSocket())
    reportConfigWarning("Could not open the AIVDM UDP socket - no AIS will be sent.");

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void TrafficSim::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
}

//---------------------------------------------------------
// Procedure: nextMmsi()

unsigned int TrafficSim::nextMmsi()
{
  m_mmsi_next++;
  return(m_mmsi_next);
}

//---------------------------------------------------------
// Procedure: shipTypeToAisCode()
//   Maps a readable type from the config onto an ITU ship-and-cargo code,
//   which is what determines the icon and label the UI shows.

int TrafficSim::shipTypeToAisCode(const string& type)
{
  string t = tolower(type);
  if(t == "tug")            return(52);
  if(t == "ferry")          return(60);
  if(t == "passenger")      return(60);
  if(t == "cargo")          return(70);
  if(t == "tanker")         return(80);
  if(t == "fishing")        return(30);
  if(t == "sailing")        return(36);
  if(t == "pleasure")       return(37);
  if(t == "pilot")          return(50);
  if(t == "sar")            return(51);
  if(t == "lawenforcement") return(55);
  return(70);
}

//---------------------------------------------------------
// Procedure: configLabel()
//   The MOOS mission reader strips whitespace out of configuration values, so
//   "TUG ALPHA" would arrive as "TUGALPHA". Underscores stand in for spaces.

static string configLabel(const string& raw)
{
  string out = raw;
  for(unsigned int i=0; i<out.size(); i++) {
    if(out[i] == '_')
      out[i] = ' ';
  }
  return(out);
}

//---------------------------------------------------------
// Procedure: handleConfigContact()
//   contact = name=TUG_ALPHA, type=tug, x=300, y=200, hdg=270, spd=4

bool TrafficSim::handleConfigContact(const string& value)
{
  Contact c;
  c.mode = MODE_TRANSIT;

  vector<string> kvs = parseString(value, ',');
  for(unsigned int i=0; i<kvs.size(); i++) {
    string kv  = stripBlankEnds(kvs[i]);
    string key = tolower(biteStringX(kv, '='));
    string val = kv;

    if(key == "name")          c.label = configLabel(val);
    else if(key == "type") {
      c.node_type    = val;
      c.ais_shiptype = shipTypeToAisCode(val);
    }
    else if(key == "callsign") c.callsign = val;
    else if(key == "x")        c.x   = atof(val.c_str());
    else if(key == "y")        c.y   = atof(val.c_str());
    else if(key == "hdg")      c.hdg = atof(val.c_str());
    else if(key == "spd")      c.spd = atof(val.c_str());
    else if(key == "length")   c.length = atof(val.c_str());
    else if(key == "beam")     c.beam   = atof(val.c_str());
    else if(key == "mmsi")     c.mmsi   = (unsigned int) atol(val.c_str());
    else if(key == "navstatus") c.navstatus = atoi(val.c_str());
    else return(false);
  }

  if(c.label == "")
    return(false);

  m_contacts.push_back(c);
  return(true);
}

//---------------------------------------------------------
// Procedure: handleConfigLoop()
//   loop = name=WORKBOAT_ECHO, type=ship, spd=5, pts=0,0:600,0:600,600:0,600
//
//   The point list is pulled out first: it is comma separated too, so it would
//   otherwise be shredded by the key=value split.

bool TrafficSim::handleConfigLoop(const string& value)
{
  string head = value;
  string pts;

  string::size_type pos = tolower(value).find("pts=");
  if(pos != string::npos) {
    head = value.substr(0, pos);
    pts  = value.substr(pos + 4);
  }

  Contact c;
  c.mode = MODE_LOOP;

  vector<string> kvs = parseString(head, ',');
  for(unsigned int i=0; i<kvs.size(); i++) {
    string kv = stripBlankEnds(kvs[i]);
    if(kv == "")
      continue;
    string key = tolower(biteStringX(kv, '='));
    string val = kv;

    if(key == "name")          c.label = configLabel(val);
    else if(key == "type") {
      c.node_type    = val;
      c.ais_shiptype = shipTypeToAisCode(val);
    }
    else if(key == "callsign") c.callsign = val;
    else if(key == "spd")      c.spd = atof(val.c_str());
    else if(key == "length")   c.length = atof(val.c_str());
    else if(key == "beam")     c.beam   = atof(val.c_str());
    else if(key == "mmsi")     c.mmsi   = (unsigned int) atol(val.c_str());
    else return(false);
  }

  vector<string> legs = parseString(pts, ':');
  for(unsigned int i=0; i<legs.size(); i++) {
    string leg = stripBlankEnds(legs[i]);
    if(leg == "")
      continue;
    vector<string> xy = parseString(leg, ',');
    if(xy.size() != 2)
      return(false);
    c.loop_x.push_back(atof(stripBlankEnds(xy[0]).c_str()));
    c.loop_y.push_back(atof(stripBlankEnds(xy[1]).c_str()));
  }

  if((c.label == "") || (c.loop_x.size() < 2))
    return(false);

  // Start on the first waypoint, aimed at the second.
  c.x = c.loop_x[0];
  c.y = c.loop_y[0];
  c.loop_idx = 1;
  c.hdg = atan2(c.loop_x[1] - c.x, c.loop_y[1] - c.y) / DEG_TO_RAD;

  m_contacts.push_back(c);
  return(true);
}

//---------------------------------------------------------
// Procedure: spawnRandomContacts()

void TrafficSim::spawnRandomContacts()
{
  for(unsigned int i=0; i<m_random_count; i++) {
    Contact c;
    c.mode  = MODE_RANDOM;
    c.label = "TRAFFIC " + uintToString(i+1);
    c.mmsi  = nextMmsi();
    c.vname = "AIS" + uintToString(c.mmsi);
    c.callsign = "SIM" + uintToString(c.mmsi % 10000);

    // A mixed fleet reads more like a real harbour than a row of identical ships.
    const char* types[] = {"cargo", "tug", "fishing", "ferry", "pleasure"};
    string type = types[i % 5];
    c.node_type    = "ship";
    c.ais_shiptype = shipTypeToAisCode(type);
    c.length = 20 + (rand() % 60);
    c.beam   = 5  + (rand() % 10);

    respawnRandom(c);
    m_contacts.push_back(c);
  }
}

//---------------------------------------------------------
// Procedure: respawnRandom()
//   Place the contact on the edge of the traffic circle, pointed roughly at
//   the USV so it actually crosses the area rather than drifting away.

void TrafficSim::respawnRandom(Contact& c)
{
  double bearing = (rand() % 3600) / 10.0;
  double radius  = m_random_radius * 0.95;

  c.x = m_ownship_x + (radius * sin(bearing * DEG_TO_RAD));
  c.y = m_ownship_y + (radius * cos(bearing * DEG_TO_RAD));

  // Inbound heading is the reciprocal, skewed so tracks are not all radial.
  double inbound = bearing + 180.0 + ((rand() % 120) - 60);
  while(inbound >= 360) inbound -= 360;
  while(inbound < 0)    inbound += 360;
  c.hdg = inbound;

  double span = m_random_spd_max - m_random_spd_min;
  if(span < 0) span = 0;
  c.spd = m_random_spd_min + ((rand() % 1000) / 1000.0) * span;
}

//---------------------------------------------------------
// Procedure: advanceContact()

void TrafficSim::advanceContact(Contact& c, double dt)
{
  if(c.mode == MODE_LOOP && !c.loop_x.empty()) {
    double tx = c.loop_x[c.loop_idx % c.loop_x.size()];
    double ty = c.loop_y[c.loop_idx % c.loop_y.size()];
    double dx = tx - c.x;
    double dy = ty - c.y;

    if(hypot(dx, dy) < 25.0) {
      c.loop_idx = (c.loop_idx + 1) % c.loop_x.size();
      tx = c.loop_x[c.loop_idx];
      ty = c.loop_y[c.loop_idx];
      dx = tx - c.x;
      dy = ty - c.y;
    }

    // Turn toward the leg at a plausible rate rather than snapping to it.
    double want = atan2(dx, dy) / DEG_TO_RAD;
    double diff = want - c.hdg;
    while(diff > 180)   diff -= 360;
    while(diff <= -180) diff += 360;

    double max_turn = MAX_TURN_RATE * dt;
    if(diff >  max_turn) diff =  max_turn;
    if(diff < -max_turn) diff = -max_turn;
    c.hdg += diff;
  }

  while(c.hdg >= 360) c.hdg -= 360;
  while(c.hdg < 0)    c.hdg += 360;

  c.x += c.spd * sin(c.hdg * DEG_TO_RAD) * dt;
  c.y += c.spd * cos(c.hdg * DEG_TO_RAD) * dt;

  if(c.mode == MODE_RANDOM && m_ownship_known) {
    if(hypot(c.x - m_ownship_x, c.y - m_ownship_y) > m_random_radius)
      respawnRandom(c);
  }
}

//---------------------------------------------------------
// Procedure: postNodeReport()
//   Field list matches pContactSpawn exactly - local X/Y, no LAT/LON - which is
//   what pContactMgrV20 and iWebSocket's own parser already consume.

void TrafficSim::postNodeReport(const Contact& c)
{
  string report = "NAME=" + c.vname + ",";
  report += "TYPE=" + c.node_type + ",";
  report += "TIME=" + doubleToString(MOOSTime(), 2) + ",";
  report += "X=" + doubleToString(c.x, 2) + ",";
  report += "Y=" + doubleToString(c.y, 2) + ",";
  report += "SPD=" + doubleToString(c.spd, 2) + ",";
  report += "HDG=" + doubleToString(c.hdg, 2) + ",";
  report += "YAW=" + doubleToString(c.hdg, 2) + ",";
  report += "DEPTH=0.0,";
  double rep_len = (m_report_length > 0) ? m_report_length : c.length;
  report += "LENGTH=" + doubleToString(rep_len, 1) + ",";
  report += "MODE=SIM";

  Notify("NODE_REPORT", report);
  m_node_reports_sent++;
}

//---------------------------------------------------------
// Procedure: postAis()
//   Position at 1 Hz and static data every 30 s, which is roughly what a real
//   class A transponder does and keeps the UDP feed calm.

void TrafficSim::postAis(Contact& c, double now)
{
  if(!m_udp_ok || !m_geodesy_ok)
    return;

  // Same projection as pMarineViewer and pNodeReporter (see USE_UTM in the
  // top-level CMakeLists). Getting this wrong does not move the contact in
  // MOOS - it stays at c.x/c.y - but it broadcasts it to Signal K about 100 m
  // from there, so the UI and the viewer disagree about where the traffic is.
  double lat = 0, lon = 0;
#ifdef USE_UTM
  if(!m_geodesy.UTM2LatLong(c.x, c.y, lat, lon))
#else
  if(!m_geodesy.LocalGrid2LatLong(c.x, c.y, lat, lon))
#endif
    return;

  if(now >= c.next_pos_post) {
    c.next_pos_post = now + m_pos_interval;

    AisPosition p;
    p.mmsi      = c.mmsi;
    p.navstatus = c.navstatus;
    p.lat       = lat;
    p.lon       = lon;
    p.sog       = c.spd * MPS_TO_KNOTS;
    p.cog       = c.hdg;
    p.hdg       = c.hdg;
    sendAis(encodeAisPosition(p));
  }

  if(now >= c.next_static_post) {
    // Stagger so the fleet does not report static data in one burst.
    c.next_static_post = now + m_static_interval;

    AisStatic s;
    s.mmsi        = c.mmsi;
    s.imo         = 0;
    s.callsign    = c.callsign;
    s.shipname    = c.label;
    s.shiptype    = c.ais_shiptype;
    s.dim_a       = (int) (c.length / 2);
    s.dim_b       = (int) (c.length - (int)(c.length / 2));
    s.dim_c       = (int) (c.beam / 2);
    s.dim_d       = (int) (c.beam - (int)(c.beam / 2));
    s.draught     = 3.0;
    s.destination = "SIMULATED";

    m_seq_id = (m_seq_id + 1) % 10;
    vector<string> parts = encodeAisStatic(s, m_seq_id);
    for(unsigned int i=0; i<parts.size(); i++)
      sendAis(parts[i]);
  }
}

//---------------------------------------------------------
// Procedure: openAisSocket()

bool TrafficSim::openAisSocket()
{
  m_udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if(m_udp_sock < 0) {
    m_udp_ok = false;
    return(false);
  }

  memset(&m_udp_addr, 0, sizeof(m_udp_addr));
  m_udp_addr.sin_family = AF_INET;
  m_udp_addr.sin_port   = htons(m_aivdm_port);
  if(inet_pton(AF_INET, m_aivdm_host.c_str(), &m_udp_addr.sin_addr) != 1) {
    close(m_udp_sock);
    m_udp_sock = -1;
    m_udp_ok   = false;
    return(false);
  }

  m_udp_ok = true;
  return(true);
}

//---------------------------------------------------------
// Procedure: sendAis()

void TrafficSim::sendAis(const string& sentence)
{
  if(!m_udp_ok)
    return;

  string line = sentence + "\r\n";
  ssize_t sent = sendto(m_udp_sock, line.c_str(), line.size(), 0,
                        (struct sockaddr*) &m_udp_addr, sizeof(m_udp_addr));
  if(sent > 0)
    m_aivdm_sent++;
  else
    reportRunWarning("Failed to send AIVDM datagram");
}

//---------------------------------------------------------
// Procedure: buildReport()

bool TrafficSim::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "             pTrafficSim                    " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Simulation : " << (m_enabled ? "ENABLED" : "DISABLED (enable_simulation=true)") << endl;
  m_msgs << "Origin     : " << doubleToString(m_origin_lat, 8) << ", "
                            << doubleToString(m_origin_lon, 8) << endl;
  m_msgs << "AIVDM      : " << m_aivdm_host << ":" << intToString(m_aivdm_port)
                            << (m_udp_ok ? "  [open]" : "  [CLOSED]") << endl;
  m_msgs << "Sent       : " << ulintToString(m_aivdm_sent) << " AIVDM, "
                            << ulintToString(m_node_reports_sent) << " NODE_REPORT" << endl;
  m_msgs << "Ownship    : " << (m_ownship_known
            ? (doubleToString(m_ownship_x,1) + ", " + doubleToString(m_ownship_y,1))
            : string("(waiting for NAV_X/NAV_Y)")) << endl;
  m_msgs << endl;

  ACTable actab(6);
  actab << "VName | Name | MMSI | X | Y | Spd/Hdg";
  actab.addHeaderLines();
  for(unsigned int i=0; i<m_contacts.size(); i++) {
    const Contact& c = m_contacts[i];
    actab << c.vname << c.label << uintToString(c.mmsi)
          << doubleToString(c.x, 1) << doubleToString(c.y, 1)
          << (doubleToString(c.spd, 1) + " / " + doubleToString(c.hdg, 0));
  }
  m_msgs << actab.getFormattedString();

  return(true);
}
