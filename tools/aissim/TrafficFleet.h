/************************************************************/
/*    NAME: Douglas Lima                                    */
/*    FILE: TrafficFleet.h                                  */
/*    DATE: 2026                                            */
/*                                                          */
/*  The AIS traffic around the USV. Same three generators    */
/*  and the same configuration syntax as src/pTrafficSim,    */
/*  so contact lines can be pasted straight across from      */
/*  simulated.moos - the difference is where the contacts    */
/*  go: pTrafficSim posts NODE_REPORT into MOOS directly,    */
/*  this emits only !AIVDM and lets the contacts come back    */
/*  the long way through Signal K and iDivisorNMEA.          */
/************************************************************/

#ifndef AISSIM_TrafficFleet_HEADER
#define AISSIM_TrafficFleet_HEADER

#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include <string>
#include <vector>

class NmeaOut;

class TrafficFleet
{
 public:
  /** How a contact decides where to go next. */
  enum MotionMode { MODE_TRANSIT, MODE_RANDOM, MODE_LOOP };

  struct Contact {
    std::string  label;        // friendly name, carried in AIS message 5
    unsigned int mmsi;
    std::string  callsign;
    int          ais_shiptype; // ITU ship-and-cargo code
    double       x, y;         // local grid, metres
    double       hdg;          // degrees true
    double       spd;          // metres/second
    double       length, beam;
    int          navstatus;

    MotionMode   mode;
    std::vector<double> loop_x, loop_y;
    unsigned int loop_idx;

    double       next_pos_post;
    double       next_static_post;

    Contact();
  };

  TrafficFleet();

  void setMmsiBase(unsigned int base)   { m_mmsi_base = base; m_mmsi_next = base; }
  void setRandomRadius(double v)        { m_random_radius = v; }
  void setRandomSpeeds(double lo, double hi) { m_spd_min = lo; m_spd_max = hi; }
  void setPosInterval(double v)         { m_pos_interval = v; }
  void setStaticInterval(double v)      { m_static_interval = v; }

  /** "name=TUG_ALPHA, type=tug, x=-6000, y=600, hdg=270, spd=4, length=25" */
  bool addContact(const std::string& spec);
  /** "name=WORKBOAT, type=cargo, spd=5, pts=-6000,200:-5600,200:-5600,800" */
  bool addLoop(const std::string& spec);
  /** n contacts placed on the edge of the traffic circle around the USV. */
  void spawnRandom(unsigned int n, double ownship_x, double ownship_y);

  void advance(double dt, double ownship_x, double ownship_y);

  /** Emit the AIS sentences that have come due. `now` is a monotonic seconds
   *  value; `geo` projects the local grid back to the lat/lon AIS carries. */
  void emit(NmeaOut& out, CMOOSGeodesy& geo, double now);

  size_t size() const { return m_contacts.size(); }
  const std::vector<Contact>& contacts() const { return m_contacts; }

 private:
  unsigned int nextMmsi() { return m_mmsi_next++; }
  void         respawnRandom(Contact& c, double ownship_x, double ownship_y);

  std::vector<Contact> m_contacts;

  unsigned int m_mmsi_base;
  unsigned int m_mmsi_next;
  double       m_random_radius;
  double       m_spd_min, m_spd_max;
  double       m_pos_interval;
  double       m_static_interval;
  int          m_seq_id;
};

/** Shared with the ownship path in main.cpp. */
int  shipTypeToAisCode(const std::string& type);
/** Underscores stand in for spaces so a name survives config parsing. */
std::string configLabel(const std::string& raw);

#endif
