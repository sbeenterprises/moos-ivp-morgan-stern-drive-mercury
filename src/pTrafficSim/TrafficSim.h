/************************************************************/
/*    NAME: DL                                              */
/*    FILE: TrafficSim.h                                    */
/*    DATE: 2026                                            */
/*                                                          */
/*  Simulates boat traffic around the USV.                  */
/*                                                          */
/*  Each contact is published twice, on purpose:            */
/*    - NODE_REPORT, so pContactMgrV20 and the helm see it  */
/*      and behaviours such as BHV_Trail can act on it;     */
/*    - !AIVDM over UDP to the Signal K server, so the UI   */
/*      sees it as a genuine AIS target rather than a       */
/*      bespoke overlay.                                    */
/*                                                          */
/*  The vessel name is derived from the MMSI (AIS<mmsi>) so  */
/*  the UI can name a contact to follow using only the MMSI  */
/*  it already has, with no lookup table anywhere.          */
/************************************************************/

#ifndef TrafficSim_HEADER
#define TrafficSim_HEADER

#include <string>
#include <vector>
#include <netinet/in.h>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"

class TrafficSim : public AppCastingMOOSApp
{
 public:
  TrafficSim();
  ~TrafficSim();

 protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  /** How a contact decides where to go next. */
  enum MotionMode { MODE_TRANSIT, MODE_RANDOM, MODE_LOOP };

  struct Contact {
    std::string  vname;        // AIS<mmsi> - what the helm and pMarineViewer see
    std::string  label;        // friendly name, carried in AIS message 5
    unsigned int mmsi;
    std::string  node_type;    // MOOS node type: ship, tug, kayak, ...
    int          ais_shiptype; // ITU ship-and-cargo code
    std::string  callsign;
    double       x, y;         // MOOS local grid, metres
    double       hdg;          // degrees true
    double       spd;          // metres/second
    double       length, beam;
    int          navstatus;

    MotionMode   mode;
    std::vector<double> loop_x, loop_y;
    unsigned int loop_idx;

    double       next_pos_post;    // next AIS message 1
    double       next_static_post; // next AIS message 5

    // Spawned at runtime (TRAFFIC_SPAWN) rather than declared in the mission
    // file. TRAFFIC_CLEAR only ever removes these, so a test target can never
    // take the configured scenario contacts with it.
    bool         runtime;

    // Still steering for an intercept. A course solved once at spawn goes stale
    // the moment the USV accelerates or turns onto its next leg, and the target
    // then sails past a few hundred metres away without ever triggering the
    // avoidance it was placed to test. So an at_ownship target keeps correcting
    // while it is far out, and locks its course at aim_lock_range - well
    // outside the behaviour's envelope, so everything the helm ever sees is a
    // steady contact holding course and speed.
    bool         aiming;

    Contact();
  };

  void   registerVariables();

  bool   parseContactSpec(const std::string& value, Contact& c);
  bool   handleConfigContact(const std::string& value);
  bool   handleConfigLoop(const std::string& value);
  bool   handleSpawnRequest(const std::string& value);
  bool   handleClearRequest(const std::string& value);
  void   postSpawnRefusal(const std::string& why);
  void   postTargetList();
  void   assignIdentity(Contact& c);
  double interceptHeading(double cx, double cy, double spd) const;
  void   spawnRandomContacts();
  void   respawnRandom(Contact& c);

  void   advanceContact(Contact& c, double dt);
  void   postNodeReport(const Contact& c);
  void   postAis(Contact& c, double now);

  bool   openAisSocket();
  void   sendAis(const std::string& sentence);

  unsigned int nextMmsi();
  unsigned int nextSpawnMmsi();
  static int   shipTypeToAisCode(const std::string& type);

 private: // Configuration
  bool         m_enabled;
  double       m_origin_lat;
  double       m_origin_lon;
  std::string  m_aivdm_host;
  int          m_aivdm_port;
  unsigned int m_mmsi_base;
  // Runtime spawns draw from their own block so a click-placed target can never
  // collide with an MMSI the mission file already handed to a scenario contact.
  unsigned int m_spawn_mmsi_base;
  // Poked with a cleared contact's name so pContactMgrV20 tells the helm to
  // expunge the avoidance behaviour immediately, instead of leaving it running
  // against a contact that has stopped reporting. Must match the mission file's
  // pContactMgrV20 expunge_var; empty disables the post.
  std::string  m_expunge_var;
  // A target placed far outside the avoidance envelope would just sit there
  // doing nothing, and a runaway caller could fill the fleet. Both refusals are
  // reported back in the ack rather than failing silently.
  double       m_spawn_max_range;
  unsigned int m_max_runtime_contacts;
  double       m_aim_lock_range;

  unsigned int m_random_count;
  double       m_random_radius;
  double       m_random_spd_min;
  double       m_random_spd_max;

  double       m_pos_interval;     // seconds between AIS message 1
  double       m_static_interval;  // seconds between AIS message 5

  // pMarineViewer scales each contact's icon by the LENGTH it reads out of
  // NODE_REPORT, so a mixed fleet renders at mixed sizes. This is the single
  // length reported to the viewer / helm; 0 falls back to each contact's own
  // length. The AIS message 5 dimensions always use the real length and beam.
  double       m_report_length;

 private: // State
  std::vector<Contact> m_contacts;
  unsigned int         m_mmsi_next;
  unsigned int         m_spawn_mmsi_next;
  double               m_ownship_x;
  double               m_ownship_y;
  // Course and speed as well as position: a target aimed at where the USV is
  // right now misses a moving boat, so hdg=at_ownship solves for an intercept.
  double               m_ownship_hdg;
  double               m_ownship_spd;
  bool                 m_ownship_known;
  double               m_last_iterate;
  bool                 m_random_spawned;

  CMOOSGeodesy m_geodesy;
  bool         m_geodesy_ok;

  int                m_udp_sock;
  struct sockaddr_in m_udp_addr;
  bool               m_udp_ok;

  unsigned long m_aivdm_sent;
  unsigned long m_node_reports_sent;
  int           m_seq_id;

  unsigned int  m_spawns_made;
  unsigned int  m_clears_made;
  std::string   m_last_spawn_msg;
};

#endif
