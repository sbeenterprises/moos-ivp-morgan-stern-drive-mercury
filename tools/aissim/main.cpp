/************************************************************/
/*    NAME: Douglas Lima                                    */
/*    FILE: main.cpp  (aissim)                              */
/*    DATE: 2026                                            */
/*                                                          */
/*  aissim - an external NMEA0183 vessel and AIS traffic     */
/*  simulator.                                               */
/*                                                          */
/*  It stands in for the boat itself. It emits the USV's      */
/*  own $GPRMC/$GPHDT/$AGRSA/$SDDPT and the surrounding       */
/*  traffic's !AIVDM into the Signal K server over UDP, and   */
/*  takes DESIRED_RUDDER / DESIRED_THRUST back out of         */
/*  iWebSocket's feed to close the control loop.              */
/*                                                          */
/*  MOOS never sees this process. It learns where the USV     */
/*  is, and what is around it, exactly the way it does on     */
/*  the water: Signal K forwards the sentences to             */
/*  iDivisorNMEA, which publishes NAV_* and NODE_REPORT.      */
/*                                                          */
/*  This is deliberately NOT a MOOS app and is never in an    */
/*  ANTLER Run list - it has to be started by hand.           */
/************************************************************/

#include "HullModel.h"
#include "NmeaOut.h"
#include "TrafficFleet.h"
#include "AisEncode.h"

#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <json/json.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

using namespace std;

static const double MPS_TO_KNOTS = 1.94384;

static volatile sig_atomic_t g_stop = 0;
static void onSignal(int) { g_stop = 1; }

//---------------------------------------------------------
// Configuration

struct Config {
  // Where the sentences go: the Signal K server's NMEA0183 UDP provider.
  string   nmea_host       = "127.0.0.1";
  int      nmea_port       = 10112;

  // Where the actuator demand comes from: iWebSocket's server.
  string   control_url     = "ws://127.0.0.1:8000";
  bool     control_enabled = true;

  // Must match the mission's LatOrigin / LongOrigin, or MOOS and the chart
  // will disagree about where everything is.
  double   origin_lat      = 29.71970895316288;
  double   origin_lon      = -91.14705165281887;

  // Ownship. start_pos mirrors uSimMarineV22's in simulated.moos.
  unsigned int ownship_mmsi = 710400014;
  string   ownship_name    = "SWIFTSHIPS USV";
  bool     ownship_ais     = false;   // also transmit ownship as an AIS target
  double   start_x         = -6289;
  double   start_y         = 400;
  double   start_heading   = 0;
  double   start_speed     = 0;
  double   turn_rate       = 95;
  double   turn_spd_loss   = 0.4;
  string   thrust_map      = "0:0, 20:3, 40:6, 60:9, 80:12, 100:15";
  double   depth           = 12.0;
  double   nav_interval    = 0.25;    // seconds between ownship sentences

  // Traffic.
  unsigned int mmsi_base   = 366999000;
  unsigned int random_count = 0;
  double   random_radius   = 1500;
  double   random_spd_min  = 2;
  double   random_spd_max  = 9;
  double   pos_interval    = 1.0;
  double   static_interval = 30.0;

  double   tick            = 0.05;    // simulation step, seconds
};

/** Trim and drop a trailing "// comment".
 *  The marker only counts at the start of the line or after whitespace, so
 *  that "control_url = ws://127.0.0.1:8000" survives intact. */
static string cleanLine(const string& raw)
{
  string s = raw;
  for(size_t c = s.find("//"); c != string::npos; c = s.find("//", c + 1)) {
    if((c == 0) || (s[c-1] == ' ') || (s[c-1] == '\t')) {
      s = s.substr(0, c);
      break;
    }
  }

  size_t a = s.find_first_not_of(" \t\r\n");
  if(a == string::npos)
    return "";
  size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

static bool asBool(const string& v)
{
  return (v == "true") || (v == "1") || (v == "yes") || (v == "on");
}

static bool loadConfig(const string& path, Config& cfg, TrafficFleet& fleet,
                       unsigned int& random_count)
{
  ifstream in(path.c_str());
  if(!in) {
    fprintf(stderr, "aissim: cannot open config file '%s'\n", path.c_str());
    return false;
  }

  // Contact lines are collected first: random contacts need the ownship start
  // position, and every contact needs the MMSI base, so nothing can be built
  // until the whole file has been read.
  vector<string> contact_specs;
  vector<string> loop_specs;

  string raw;
  int lineno = 0;
  while(getline(in, raw)) {
    lineno++;
    string line = cleanLine(raw);
    if(line.empty())
      continue;

    size_t eq = line.find('=');
    if(eq == string::npos) {
      fprintf(stderr, "aissim: %s:%d: not a key=value line: %s\n",
              path.c_str(), lineno, line.c_str());
      return false;
    }

    string key = cleanLine(line.substr(0, eq));
    string val = cleanLine(line.substr(eq + 1));
    for(size_t i = 0; i < key.size(); i++)
      key[i] = (char) tolower((unsigned char) key[i]);

    if(key == "nmea_host")            cfg.nmea_host = val;
    else if(key == "nmea_port")       cfg.nmea_port = atoi(val.c_str());
    else if(key == "control_url")     cfg.control_url = val;
    else if(key == "control_enabled") cfg.control_enabled = asBool(val);
    else if(key == "origin_lat")      cfg.origin_lat = atof(val.c_str());
    else if(key == "origin_lon")      cfg.origin_lon = atof(val.c_str());
    else if(key == "ownship_mmsi")    cfg.ownship_mmsi = (unsigned int) atol(val.c_str());
    else if(key == "ownship_name")    cfg.ownship_name = configLabel(val);
    else if(key == "ownship_ais")     cfg.ownship_ais = asBool(val);
    else if(key == "start_x")         cfg.start_x = atof(val.c_str());
    else if(key == "start_y")         cfg.start_y = atof(val.c_str());
    else if(key == "start_heading")   cfg.start_heading = atof(val.c_str());
    else if(key == "start_speed")     cfg.start_speed = atof(val.c_str());
    else if(key == "turn_rate")       cfg.turn_rate = atof(val.c_str());
    else if(key == "turn_spd_loss")   cfg.turn_spd_loss = atof(val.c_str());
    else if(key == "thrust_map")      cfg.thrust_map = val;
    else if(key == "depth")           cfg.depth = atof(val.c_str());
    else if(key == "nav_interval")    cfg.nav_interval = atof(val.c_str());
    else if(key == "mmsi_base")       cfg.mmsi_base = (unsigned int) atol(val.c_str());
    else if(key == "random_contacts") cfg.random_count = (unsigned int) atoi(val.c_str());
    else if(key == "random_radius")   cfg.random_radius = atof(val.c_str());
    else if(key == "random_spd_min")  cfg.random_spd_min = atof(val.c_str());
    else if(key == "random_spd_max")  cfg.random_spd_max = atof(val.c_str());
    else if(key == "position_interval") cfg.pos_interval = atof(val.c_str());
    else if(key == "static_interval") cfg.static_interval = atof(val.c_str());
    else if(key == "tick")            cfg.tick = atof(val.c_str());
    else if(key == "contact")         contact_specs.push_back(val);
    else if(key == "loop")            loop_specs.push_back(val);
    else {
      fprintf(stderr, "aissim: %s:%d: unknown parameter '%s'\n",
              path.c_str(), lineno, key.c_str());
      return false;
    }
  }

  fleet.setMmsiBase(cfg.mmsi_base + 1);
  fleet.setRandomRadius(cfg.random_radius);
  fleet.setRandomSpeeds(cfg.random_spd_min, cfg.random_spd_max);
  fleet.setPosInterval(cfg.pos_interval);
  fleet.setStaticInterval(cfg.static_interval);

  for(size_t i = 0; i < contact_specs.size(); i++) {
    if(!fleet.addContact(contact_specs[i])) {
      fprintf(stderr, "aissim: bad contact line: %s\n", contact_specs[i].c_str());
      return false;
    }
  }
  for(size_t i = 0; i < loop_specs.size(); i++) {
    if(!fleet.addLoop(loop_specs[i])) {
      fprintf(stderr, "aissim: bad loop line: %s\n", loop_specs[i].c_str());
      return false;
    }
  }

  random_count = cfg.random_count;
  return true;
}

//---------------------------------------------------------
// The control link
//
// iWebSocket (src/iWebSocket/main.cpp:23) serves a plain WebSocket on port
// 8000 and pushes a JSON blob at 20 Hz carrying, among much else,
// DESIRED_RUDDER and DESIRED_THRUST (WebSocket.cpp:448-449). Those are the
// same two values iCAN turns into CAN frames on the real boat, so reading
// them here is exactly standing in for the actuators.
//
// This is receive only. aissim never sends on the socket, so it cannot
// perturb the helm.

class ControlLink
{
 public:
  ControlLink() : m_rudder(0), m_thrust(0), m_updates(0), m_connected(false) {}

  void start(const string& url)
  {
    m_ws.setUrl(url);
    m_ws.setPingInterval(20);

    m_ws.setOnMessageCallback([this](const ix::WebSocketMessagePtr& msg) {
      if(msg->type == ix::WebSocketMessageType::Open) {
        m_connected = true;
        fprintf(stderr, "aissim: control link up\n");
      }
      else if(msg->type == ix::WebSocketMessageType::Close) {
        m_connected = false;
        fprintf(stderr, "aissim: control link down\n");
      }
      else if(msg->type == ix::WebSocketMessageType::Error) {
        m_connected = false;
      }
      else if(msg->type == ix::WebSocketMessageType::Message) {
        Json::CharReaderBuilder builder;
        Json::Value root;
        string errs;
        istringstream ss(msg->str);
        if(!Json::parseFromStream(builder, ss, &root, &errs))
          return;

        if(root.isMember("DESIRED_RUDDER"))
          m_rudder = root["DESIRED_RUDDER"].asDouble();
        if(root.isMember("DESIRED_THRUST"))
          m_thrust = root["DESIRED_THRUST"].asDouble();
        m_updates++;
      }
    });

    m_ws.start();   // reconnects on its own if iWebSocket restarts
  }

  void stop() { m_ws.stop(); }

  double rudder() const { return m_rudder; }
  double thrust() const { return m_thrust; }
  bool   connected() const { return m_connected; }
  unsigned long updates() const { return m_updates; }

 private:
  ix::WebSocket        m_ws;
  atomic<double>       m_rudder;
  atomic<double>       m_thrust;
  atomic<unsigned long> m_updates;
  atomic<bool>         m_connected;
};

//---------------------------------------------------------
// main

static double nowSeconds()
{
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return ts.tv_sec + (ts.tv_nsec / 1e9);
}

int main(int argc, char* argv[])
{
  string cfg_path = "aissim.cfg";
  for(int i = 1; i < argc; i++) {
    string arg = argv[i];
    if((arg == "-h") || (arg == "--help")) {
      printf("Usage: aissim [config-file]\n"
             "  Default config file: aissim.cfg\n"
             "  Emits ownship NMEA0183 and AIS traffic to the Signal K server,\n"
             "  and reads DESIRED_RUDDER / DESIRED_THRUST back from iWebSocket.\n");
      return 0;
    }
    cfg_path = arg;
  }

  Config       cfg;
  TrafficFleet fleet;
  unsigned int random_count = 0;

  if(!loadConfig(cfg_path, cfg, fleet, random_count))
    return 1;

  CMOOSGeodesy geo;
  if(!geo.Initialise(cfg.origin_lat, cfg.origin_lon)) {
    fprintf(stderr, "aissim: geodesy failed to initialise\n");
    return 1;
  }

  HullModel hull;
  if(!hull.setThrustMap(cfg.thrust_map)) {
    fprintf(stderr, "aissim: bad thrust_map '%s'\n", cfg.thrust_map.c_str());
    return 1;
  }
  hull.setTurnRate(cfg.turn_rate);
  hull.setTurnSpdLoss(cfg.turn_spd_loss);
  hull.setPose(cfg.start_x, cfg.start_y, cfg.start_heading, cfg.start_speed);

  if(random_count > 0)
    fleet.spawnRandom(random_count, cfg.start_x, cfg.start_y);

  NmeaOut out;
  if(!out.open(cfg.nmea_host, cfg.nmea_port)) {
    fprintf(stderr, "aissim: cannot open UDP socket to %s:%d\n",
            cfg.nmea_host.c_str(), cfg.nmea_port);
    return 1;
  }

  ix::initNetSystem();
  ControlLink control;
  if(cfg.control_enabled)
    control.start(cfg.control_url);
  else
    fprintf(stderr, "aissim: control link disabled - the USV will not respond "
                    "to the helm\n");

  signal(SIGINT,  onSignal);
  signal(SIGTERM, onSignal);

  printf("aissim: %zu AIS contacts -> %s:%d\n",
         fleet.size(), cfg.nmea_host.c_str(), cfg.nmea_port);
  printf("aissim: ownship MMSI %u, control from %s\n",
         cfg.ownship_mmsi, cfg.control_enabled ? cfg.control_url.c_str() : "(none)");
  printf("aissim: origin %.8f / %.8f - this MUST match the mission's "
         "LatOrigin / LongOrigin\n", cfg.origin_lat, cfg.origin_lon);

  double t_start    = nowSeconds();
  double t_last     = t_start;
  double next_nav   = 0;
  double next_ais   = 0;   // ownship's own AIS position report
  double next_print = 0;

  while(!g_stop) {
    double now = nowSeconds();
    double dt  = now - t_last;
    if(dt <= 0)
      dt = cfg.tick;
    t_last = now;

    double rudder = control.rudder();
    double thrust = control.thrust();

    hull.step(dt, rudder, thrust);
    fleet.advance(dt, hull.x(), hull.y());

    // ---- ownship sentences ----
    if(now >= next_nav) {
      next_nav = now + cfg.nav_interval;

      double lat = 0, lon = 0;
#ifdef USE_UTM
      if(geo.UTM2LatLong(hull.x(), hull.y(), lat, lon)) {
#else
      if(geo.LocalGrid2LatLong(hull.x(), hull.y(), lat, lon)) {
#endif
        out.send(buildGPRMC(now, lat, lon,
                            hull.speed() * MPS_TO_KNOTS, hull.heading()));
        out.send(buildGPHDT(hull.heading()));
        // The rudder demand doubles as the rudder feedback: there is no
        // steering gear lag in this model.
        out.send(buildAGRSA(rudder));
        out.send(buildSDDPT(cfg.depth));

        if(cfg.ownship_ais && (now >= next_ais)) {
          next_ais = now + cfg.pos_interval;
          AisPosition p;
          p.mmsi = cfg.ownship_mmsi;
          p.lat  = lat;
          p.lon  = lon;
          p.sog  = hull.speed() * MPS_TO_KNOTS;
          p.cog  = hull.heading();
          p.hdg  = hull.heading();
          out.send(encodeAisPosition(p));
        }
      }
    }

    // ---- traffic ----
    fleet.emit(out, geo, now);

    if(now >= next_print) {
      next_print = now + 5.0;
      printf("t=%.0f  x=%.1f y=%.1f hdg=%.1f spd=%.2f | rudder=%.1f thrust=%.1f"
             " | control=%s | sent=%u\n",
             now - t_start, hull.x(), hull.y(), hull.heading(),
             hull.speed(), rudder, thrust,
             control.connected() ? "up" : "DOWN", out.sent());
      fflush(stdout);
    }

    long sleep_us = (long)(cfg.tick * 1e6);
    if(sleep_us > 0)
      this_thread::sleep_for(chrono::microseconds(sleep_us));
  }

  printf("\naissim: stopping\n");
  if(cfg.control_enabled)
    control.stop();
  ix::uninitNetSystem();
  return 0;
}
