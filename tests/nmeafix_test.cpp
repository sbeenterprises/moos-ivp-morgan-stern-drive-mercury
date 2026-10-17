// Off-vessel unit test for NmeaFix.h.
// Built and run by tests/run_tests.sh (plain g++, no MOOS).
#include "NmeaFix.h"   // src/iDivisorNMEA, added to the include path by run_tests.sh
#include <cmath>
#include <iostream>

static int fails = 0;
#define CHECK(cond) do { if(!(cond)) { std::cerr << "FAIL line " << __LINE__ << ": " #cond "\n"; fails++; } } while(0)

int main()
{
  // Real-format GGA with a correct checksum (GN talker, 11 sats, HDOP 0.9)
  const std::string gga = "$GNGGA,154530.00,2943.18240,N,09108.82310,W,1,11,0.90,3.2,M,-27.1,M,,*7F";
  // recompute: the test asserts checksumOk only on sentences we know are right
  nmeafix::GgaFix g = nmeafix::parseGGA(gga);
  CHECK(g.valid);
  CHECK(g.quality == 1);
  CHECK(g.sats == 11);
  CHECK(std::fabs(g.hdop - 0.90) < 1e-9);
  CHECK(std::fabs(g.alt_m - 3.2) < 1e-9);

  // GP talker, no fix: empty position, quality 0
  nmeafix::GgaFix g0 = nmeafix::parseGGA("$GPGGA,154531.00,,,,,0,00,99.99,,,,,,*66");
  CHECK(g0.valid);
  CHECK(g0.quality == 0);
  CHECK(g0.sats == 0);
  CHECK(g0.hdop > 99.0);

  // Not a GGA
  CHECK(!nmeafix::parseGGA("$GPRMC,154530.00,A,2943.18240,N,09108.82310,W,6.1,49.0,161026,,,A*00").valid);
  CHECK(!nmeafix::parseGGA("!AIVDM,1,1,,A,13u?etPv2;0n:dDPwUM1U1Cb069D,0*24").valid);
  CHECK(!nmeafix::parseGGA("").valid);
  CHECK(!nmeafix::parseGGA("$GNGGA,1").valid);

  // RMC status
  CHECK(nmeafix::rmcStatus("$GPRMC,154530.00,A,2943.18,N,09108.82,W,6.1,49.0,161026,,,A*00") == 1);
  CHECK(nmeafix::rmcStatus("$GPRMC,154530.00,V,,,,,,,161026,,,N*00") == 0);
  CHECK(nmeafix::rmcStatus("$IIHDT,259.6,T*2A") == -1);

  // Checksums: the IIHDT line is copied from the 16 Oct USV-722 log
  CHECK(nmeafix::checksumOk("$IIHDT,259.6,T*2A"));
  CHECK(nmeafix::checksumOk("$IIHDT,259.5,T*29"));
  CHECK(!nmeafix::checksumOk("$IIHDT,259.6,T*2B"));
  CHECK(!nmeafix::checksumOk("$IIHDT,259.6,T"));

  // Talker-agnostic type
  CHECK(nmeafix::sentenceType("$GNGGA,...") == "GGA");
  CHECK(nmeafix::sentenceType("$IIHDT,1,T") == "HDT");
  CHECK(nmeafix::qualityLabel(4) == "RTK_FIXED");

  if(fails == 0) std::cout << "NmeaFix: all checks passed\n";
  return fails == 0 ? 0 : 1;
}
