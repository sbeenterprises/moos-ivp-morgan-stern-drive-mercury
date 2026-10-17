/************************************************************/
/*    FILE: NmeaFix.h                                       */
/*    Small, dependency-free helpers for GNSS fix quality.  */
/*    Kept header-only and free of MOOS types so it can be  */
/*    unit tested off the vessel (see test_NmeaFix.cpp).    */
/************************************************************/

#ifndef NMEA_FIX_HEADER
#define NMEA_FIX_HEADER

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace nmeafix {

// Split an NMEA sentence into comma fields. The "*hh" checksum is stripped
// from the last field. Empty fields are kept so field indexes stay stable.
inline std::vector<std::string> fields(const std::string& sentence)
{
  std::string s = sentence;
  // drop trailing CR/LF
  while(!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' '))
    s.pop_back();
  size_t star = s.find('*');
  if(star != std::string::npos)
    s = s.substr(0, star);

  std::vector<std::string> out;
  std::string cur;
  for(char c : s) {
    if(c == ',') { out.push_back(cur); cur.clear(); }
    else cur.push_back(c);
  }
  out.push_back(cur);
  return out;
}

// True when the NMEA checksum is present and correct. A sentence without a
// checksum returns false: GGA/RMC from any normal receiver always carry one.
inline bool checksumOk(const std::string& sentence)
{
  size_t start = sentence.find_first_of("$!");
  size_t star  = sentence.find('*');
  if(start == std::string::npos || star == std::string::npos || star + 3 > sentence.size())
    return false;
  unsigned char sum = 0;
  for(size_t i = start + 1; i < star; i++)
    sum ^= (unsigned char)sentence[i];
  unsigned int given = 0;
  if(std::sscanf(sentence.c_str() + star + 1, "%2x", &given) != 1)
    return false;
  return sum == (unsigned char)given;
}

// Talker-agnostic sentence type: "$GPGGA", "$GNGGA", "$IIGGA" -> "GGA".
inline std::string sentenceType(const std::string& sentence)
{
  if(sentence.size() < 6 || (sentence[0] != '$' && sentence[0] != '!'))
    return "";
  return sentence.substr(3, 3);
}

struct GgaFix {
  bool   valid    = false;  // sentence parsed (not the same as "has a fix")
  int    quality  = 0;      // 0 none, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float, 6 dead reckoning
  int    sats     = 0;      // satellites used in the solution
  double hdop     = 99.9;   // horizontal dilution of precision
  double alt_m    = 0.0;    // antenna altitude above MSL
};

// GGA field layout:
//   0 $xxGGA, 1 hhmmss.ss, 2 lat, 3 N/S, 4 lon, 5 E/W,
//   6 quality, 7 sats used, 8 HDOP, 9 altitude, 10 M, ...
inline GgaFix parseGGA(const std::string& sentence)
{
  GgaFix g;
  if(sentenceType(sentence) != "GGA")
    return g;
  std::vector<std::string> f = fields(sentence);
  if(f.size() < 10)
    return g;
  g.quality = f[6].empty() ? 0    : std::atoi(f[6].c_str());
  g.sats    = f[7].empty() ? 0    : std::atoi(f[7].c_str());
  g.hdop    = f[8].empty() ? 99.9 : std::atof(f[8].c_str());
  g.alt_m   = f[9].empty() ? 0.0  : std::atof(f[9].c_str());
  g.valid   = true;
  return g;
}

// RMC field 2 is the status: 'A' = data valid, 'V' = receiver warning.
// Returns 1 valid, 0 void, -1 not an RMC / unparseable.
inline int rmcStatus(const std::string& sentence)
{
  if(sentenceType(sentence) != "RMC")
    return -1;
  std::vector<std::string> f = fields(sentence);
  if(f.size() < 3 || f[2].empty())
    return -1;
  return (f[2][0] == 'A') ? 1 : 0;
}

// Human readable quality label for the appcast and the UI.
inline std::string qualityLabel(int q)
{
  switch(q) {
    case 0: return "NO_FIX";
    case 1: return "GPS";
    case 2: return "DGPS";
    case 3: return "PPS";
    case 4: return "RTK_FIXED";
    case 5: return "RTK_FLOAT";
    case 6: return "DEAD_RECKONING";
    default: return "UNKNOWN";
  }
}

} // namespace nmeafix

#endif
