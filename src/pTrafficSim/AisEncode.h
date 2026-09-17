/************************************************************/
/*    NAME: DL                                              */
/*    FILE: AisEncode.h                                     */
/*    DATE: 2026                                            */
/*                                                          */
/*  Minimal AIVDM encoder for simulated AIS traffic.        */
/*                                                          */
/*  Only what pTrafficSim needs: message type 1 (position   */
/*  report) and type 5 (static and voyage data). Bit        */
/*  layouts follow gpsd/ITU-R M.1371 and were cross-checked */
/*  against the ggencoder library that the Signal K server  */
/*  actually decodes with, so the sentences produced here   */
/*  are known to round-trip through it.                     */
/************************************************************/

#ifndef AisEncode_HEADER
#define AisEncode_HEADER

#include <string>
#include <vector>

/** AIS message 1 - position report (class A) */
struct AisPosition {
  unsigned int mmsi;
  int          navstatus;   // 0 = under way using engine, 1 = anchored, 5 = moored
  double       lat;         // degrees, +N
  double       lon;         // degrees, +E
  double       sog;         // knots
  double       cog;         // degrees true
  double       hdg;         // degrees true (511 = not available)

  AisPosition() : mmsi(0), navstatus(0), lat(0), lon(0), sog(0), cog(0), hdg(511) {}
};

/** AIS message 5 - static and voyage related data (class A) */
struct AisStatic {
  unsigned int mmsi;
  unsigned int imo;
  std::string  callsign;    // <= 7 chars
  std::string  shipname;    // <= 20 chars
  int          shiptype;    // ITU ship-and-cargo code, e.g. 52 = tug, 60 = passenger
  int          dim_a;       // metres bow of antenna
  int          dim_b;       // metres stern of antenna
  int          dim_c;       // metres port of antenna
  int          dim_d;       // metres starboard of antenna
  double       draught;     // metres
  std::string  destination; // <= 20 chars

  AisStatic() : mmsi(0), imo(0), shiptype(0),
                dim_a(0), dim_b(0), dim_c(0), dim_d(0), draught(0) {}
};

/** One complete sentence, e.g. "!AIVDM,1,1,,A,<payload>,0*5C" */
std::string encodeAisPosition(const AisPosition& p);

/** Two sentences - message 5 is 424 bits and does not fit one sentence.
 *  seq_id (0-9) ties the fragments together for the receiver. */
std::vector<std::string> encodeAisStatic(const AisStatic& s, int seq_id);

#endif
