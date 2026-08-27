/************************************************************/
/*    NAME: Douglas Lima                                    */
/*    FILE: NmeaOut.cpp                                     */
/*    DATE: 2026                                            */
/************************************************************/

#include "NmeaOut.h"
#include <arpa/inet.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <sys/socket.h>
#include <unistd.h>

using namespace std;

//---------------------------------------------------------
// NmeaOut

NmeaOut::NmeaOut() : m_sock(-1), m_sent(0)
{
  memset(&m_addr, 0, sizeof(m_addr));
}

NmeaOut::~NmeaOut()
{
  if(m_sock >= 0)
    close(m_sock);
}

bool NmeaOut::open(const string& host, int port)
{
  m_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if(m_sock < 0)
    return false;

  m_addr.sin_family = AF_INET;
  m_addr.sin_port   = htons(port);
  if(inet_pton(AF_INET, host.c_str(), &m_addr.sin_addr) != 1) {
    close(m_sock);
    m_sock = -1;
    return false;
  }
  return true;
}

bool NmeaOut::send(const string& sentence)
{
  if(m_sock < 0)
    return false;

  // One sentence per datagram, CRLF terminated the way a serial talker would.
  string wire = sentence + "\r\n";
  ssize_t n = sendto(m_sock, wire.c_str(), wire.size(), 0,
                     (struct sockaddr*) &m_addr, sizeof(m_addr));
  if(n < 0)
    return false;

  m_sent++;
  return true;
}

//---------------------------------------------------------
// Sentence builders

namespace {

/** Degrees to the NMEA ddmm.mmmm / dddmm.mmmm form plus its hemisphere. */
void degToNmea(double deg, bool is_lat, string& field, string& hemi)
{
  if(is_lat)
    hemi = (deg < 0) ? "S" : "N";
  else
    hemi = (deg < 0) ? "W" : "E";

  double a = fabs(deg);
  int    d = (int) a;
  double m = (a - d) * 60.0;

  char buf[32];
  if(is_lat)
    snprintf(buf, sizeof(buf), "%02d%07.4f", d, m);
  else
    snprintf(buf, sizeof(buf), "%03d%07.4f", d, m);
  field = buf;
}

} // anonymous namespace

string nmeaFrame(const string& body)
{
  // Checksum is the XOR of everything between the leading '$' / '!' and '*'.
  unsigned char cs = 0;
  for(size_t i = 1; i < body.size(); i++)
    cs = (unsigned char)(cs ^ (unsigned char) body[i]);

  char buf[8];
  snprintf(buf, sizeof(buf), "*%02X", cs);
  return body + buf;
}

string buildGPRMC(double utc_secs, double lat, double lon,
                  double sog, double cog)
{
  string lat_f, lat_h, lon_f, lon_h;
  degToNmea(lat, true,  lat_f, lat_h);
  degToNmea(lon, false, lon_f, lon_h);

  time_t    t = (time_t) utc_secs;
  struct tm g;
  gmtime_r(&t, &g);

  double frac = utc_secs - floor(utc_secs);

  char buf[160];
  snprintf(buf, sizeof(buf),
           "$GPRMC,%02d%02d%05.2f,A,%s,%s,%s,%s,%.2f,%.1f,%02d%02d%02d,,",
           g.tm_hour, g.tm_min, g.tm_sec + frac,
           lat_f.c_str(), lat_h.c_str(),
           lon_f.c_str(), lon_h.c_str(),
           sog, cog,
           g.tm_mday, g.tm_mon + 1, (g.tm_year + 1900) % 100);

  return nmeaFrame(string(buf));
}

string buildGPHDT(double heading)
{
  char buf[48];
  snprintf(buf, sizeof(buf), "$GPHDT,%.2f,T", heading);
  return nmeaFrame(string(buf));
}

string buildAGRSA(double rudder_deg)
{
  char buf[48];
  snprintf(buf, sizeof(buf), "$AGRSA,%.1f,A,,V", rudder_deg);
  return nmeaFrame(string(buf));
}

string buildSDDPT(double depth_m)
{
  char buf[48];
  snprintf(buf, sizeof(buf), "$SDDPT,%.1f,0.0", depth_m);
  return nmeaFrame(string(buf));
}
