/************************************************************/
/*    NAME: Douglas Lima                                    */
/*    FILE: TrafficFleet.cpp                                */
/*    DATE: 2026                                            */
/************************************************************/

#include "TrafficFleet.h"
#include "NmeaOut.h"
#include "AisEncode.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

using namespace std;

static const double MPS_TO_KNOTS  = 1.94384;
static const double DEG_TO_RAD    = M_PI / 180.0;
static const double MAX_TURN_RATE = 15.0;   // degrees/second, keeps loops plausible

//---------------------------------------------------------
// Small helpers

int shipTypeToAisCode(const string& type)
{
  string t;
  for(size_t i = 0; i < type.size(); i++)
    t += (char) tolower((unsigned char) type[i]);

  if(t == "tug")            return 52;
  if(t == "ferry")          return 60;
  if(t == "passenger")      return 60;
  if(t == "cargo")          return 70;
  if(t == "tanker")         return 80;
  if(t == "fishing")        return 30;
  if(t == "sailing")        return 36;
  if(t == "pleasure")       return 37;
  if(t == "pilot")          return 50;
  if(t == "sar")            return 51;
  if(t == "lawenforcement") return 55;
  return 70;
}

string configLabel(const string& raw)
{
  string out = raw;
  for(size_t i = 0; i < out.size(); i++) {
    if(out[i] == '_')
      out[i] = ' ';
  }
  return out;
}

namespace {

string stripEnds(const string& s)
{
  size_t a = s.find_first_not_of(" \t\r\n");
  if(a == string::npos)
    return "";
  size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

vector<string> splitOn(const string& s, char delim)
{
  vector<string> out;
  stringstream ss(s);
  string part;
  while(getline(ss, part, delim))
    out.push_back(part);
  return out;
}

string lowerOf(const string& s)
{
  string out;
  for(size_t i = 0; i < s.size(); i++)
    out += (char) tolower((unsigned char) s[i]);
  return out;
}

/** Split "key=value" at the first '='. */
void biteKey(const string& kv, string& key, string& val)
{
  size_t eq = kv.find('=');
  if(eq == string::npos) {
    key = lowerOf(stripEnds(kv));
    val = "";
    return;
  }
  key = lowerOf(stripEnds(kv.substr(0, eq)));
  val = stripEnds(kv.substr(eq + 1));
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
// Contact

TrafficFleet::Contact::Contact()
{
  mmsi = 0;
  ais_shiptype = 70;
  x = 0; y = 0; hdg = 0; spd = 0;
  length = 30; beam = 8;
  navstatus = 0;
  mode = MODE_TRANSIT;
  loop_idx = 0;
  next_pos_post = 0;
  next_static_post = 0;
}

//---------------------------------------------------------
// TrafficFleet

TrafficFleet::TrafficFleet()
{
  m_mmsi_base       = 366999000;
  m_mmsi_next       = 366999001;
  m_random_radius   = 1500;
  m_spd_min         = 2;
  m_spd_max         = 9;
  m_pos_interval    = 1.0;
  m_static_interval = 30.0;
  m_seq_id          = 0;
}

//---------------------------------------------------------
// Procedure: addContact()

bool TrafficFleet::addContact(const string& spec)
{
  Contact c;
  c.mode = MODE_TRANSIT;

  vector<string> kvs = splitOn(spec, ',');
  for(size_t i = 0; i < kvs.size(); i++) {
    string kv = stripEnds(kvs[i]);
    if(kv.empty())
      continue;

    string key, val;
    biteKey(kv, key, val);

    if(key == "name")           c.label = configLabel(val);
    else if(key == "type")      c.ais_shiptype = shipTypeToAisCode(val);
    else if(key == "callsign")  c.callsign = val;
    else if(key == "x")         c.x = atof(val.c_str());
    else if(key == "y")         c.y = atof(val.c_str());
    else if(key == "hdg")       c.hdg = atof(val.c_str());
    else if(key == "spd")       c.spd = atof(val.c_str());
    else if(key == "length")    c.length = atof(val.c_str());
    else if(key == "beam")      c.beam = atof(val.c_str());
    else if(key == "mmsi")      c.mmsi = (unsigned int) atol(val.c_str());
    else if(key == "navstatus") c.navstatus = atoi(val.c_str());
    else return false;
  }

  if(c.label.empty())
    return false;
  if(c.mmsi == 0)
    c.mmsi = nextMmsi();
  if(c.callsign.empty()) {
    char buf[16];
    snprintf(buf, sizeof(buf), "SIM%04u", c.mmsi % 10000);
    c.callsign = buf;
  }

  m_contacts.push_back(c);
  return true;
}

//---------------------------------------------------------
// Procedure: addLoop()
//   The point list is pulled out first: it is comma separated too, so it
//   would otherwise be shredded by the key=value split.

bool TrafficFleet::addLoop(const string& spec)
{
  string head = spec;
  string pts;

  string::size_type pos = lowerOf(spec).find("pts=");
  if(pos != string::npos) {
    head = spec.substr(0, pos);
    pts  = spec.substr(pos + 4);
  }

  Contact c;
  c.mode = MODE_LOOP;

  vector<string> kvs = splitOn(head, ',');
  for(size_t i = 0; i < kvs.size(); i++) {
    string kv = stripEnds(kvs[i]);
    if(kv.empty())
      continue;

    string key, val;
    biteKey(kv, key, val);

    if(key == "name")          c.label = configLabel(val);
    else if(key == "type")     c.ais_shiptype = shipTypeToAisCode(val);
    else if(key == "callsign") c.callsign = val;
    else if(key == "spd")      c.spd = atof(val.c_str());
    else if(key == "length")   c.length = atof(val.c_str());
    else if(key == "beam")     c.beam = atof(val.c_str());
    else if(key == "mmsi")     c.mmsi = (unsigned int) atol(val.c_str());
    else return false;
  }

  vector<string> legs = splitOn(pts, ':');
  for(size_t i = 0; i < legs.size(); i++) {
    string leg = stripEnds(legs[i]);
    if(leg.empty())
      continue;
    vector<string> xy = splitOn(leg, ',');
    if(xy.size() != 2)
      return false;
    c.loop_x.push_back(atof(stripEnds(xy[0]).c_str()));
    c.loop_y.push_back(atof(stripEnds(xy[1]).c_str()));
  }

  if(c.label.empty() || (c.loop_x.size() < 2))
    return false;

  if(c.mmsi == 0)
    c.mmsi = nextMmsi();
  if(c.callsign.empty()) {
    char buf[16];
    snprintf(buf, sizeof(buf), "SIM%04u", c.mmsi % 10000);
    c.callsign = buf;
  }

  // Start on the first waypoint, aimed at the second.
  c.x = c.loop_x[0];
  c.y = c.loop_y[0];
  c.loop_idx = 1;
  c.hdg = angle360(atan2(c.loop_x[1] - c.x, c.loop_y[1] - c.y) / DEG_TO_RAD);

  m_contacts.push_back(c);
  return true;
}

//---------------------------------------------------------
// Procedure: spawnRandom()

void TrafficFleet::spawnRandom(unsigned int n, double ownship_x, double ownship_y)
{
  for(unsigned int i = 0; i < n; i++) {
    Contact c;
    c.mode = MODE_RANDOM;

    char buf[32];
    snprintf(buf, sizeof(buf), "TRAFFIC %u", i + 1);
    c.label = buf;

    c.mmsi = nextMmsi();
    snprintf(buf, sizeof(buf), "SIM%04u", c.mmsi % 10000);
    c.callsign = buf;

    // A mixed fleet reads more like a real harbour than a row of identical ships.
    const char* types[] = {"cargo", "tug", "fishing", "ferry", "pleasure"};
    c.ais_shiptype = shipTypeToAisCode(types[i % 5]);
    c.length = 20 + (rand() % 60);
    c.beam   = 5  + (rand() % 10);

    respawnRandom(c, ownship_x, ownship_y);
    m_contacts.push_back(c);
  }
}

//---------------------------------------------------------
// Procedure: respawnRandom()
//   Place the contact on the edge of the traffic circle, pointed roughly at
//   the USV so it actually crosses the area rather than drifting away.

void TrafficFleet::respawnRandom(Contact& c, double ownship_x, double ownship_y)
{
  double bearing = (rand() % 3600) / 10.0;
  double radius  = m_random_radius * 0.95;

  c.x = ownship_x + (radius * sin(bearing * DEG_TO_RAD));
  c.y = ownship_y + (radius * cos(bearing * DEG_TO_RAD));

  // Inbound heading is the reciprocal, skewed so tracks are not all radial.
  c.hdg = angle360(bearing + 180.0 + ((rand() % 120) - 60));

  double span = m_spd_max - m_spd_min;
  if(span < 0) span = 0;
  c.spd = m_spd_min + ((rand() % 1000) / 1000.0) * span;
}

//---------------------------------------------------------
// Procedure: advance()

void TrafficFleet::advance(double dt, double ownship_x, double ownship_y)
{
  for(size_t i = 0; i < m_contacts.size(); i++) {
    Contact& c = m_contacts[i];

    if((c.mode == MODE_LOOP) && !c.loop_x.empty()) {
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

    c.hdg = angle360(c.hdg);
    c.x += c.spd * sin(c.hdg * DEG_TO_RAD) * dt;
    c.y += c.spd * cos(c.hdg * DEG_TO_RAD) * dt;

    if(c.mode == MODE_RANDOM) {
      if(hypot(c.x - ownship_x, c.y - ownship_y) > m_random_radius)
        respawnRandom(c, ownship_x, ownship_y);
    }
  }
}

//---------------------------------------------------------
// Procedure: emit()
//   Position at 1 Hz and static data every 30 s per contact - roughly what a
//   real class A transponder does, and calm enough not to flood Signal K.

void TrafficFleet::emit(NmeaOut& out, CMOOSGeodesy& geo, double now)
{
  for(size_t i = 0; i < m_contacts.size(); i++) {
    Contact& c = m_contacts[i];

    double lat = 0, lon = 0;
#ifdef USE_UTM
    if(!geo.UTM2LatLong(c.x, c.y, lat, lon))
#else
    if(!geo.LocalGrid2LatLong(c.x, c.y, lat, lon))
#endif
      continue;

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
      out.send(encodeAisPosition(p));
    }

    if(now >= c.next_static_post) {
      // Stagger so the fleet does not report static data in one burst.
      c.next_static_post = now + m_static_interval + (double)(i % 10);

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
      for(size_t k = 0; k < parts.size(); k++)
        out.send(parts[k]);
    }
  }
}
