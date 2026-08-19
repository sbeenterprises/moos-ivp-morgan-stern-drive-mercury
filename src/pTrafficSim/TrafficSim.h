/************************************************************/
/*    NAME: Douglas Lima                                    */
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

    Contact();
  };

  void   registerVariables();

  bool   handleConfigContact(const std::string& value);
  bool   handleConfigLoop(const std::string& value);
  void   spawnRandomContacts();
  void   respawnRandom(Contact& c);

  void   advanceContact(Contact& c, double dt);
  void   postNodeReport(const Contact& c);
  void   postAis(Contact& c, double now);

  bool   openAisSocket();
  void   sendAis(const std::string& sentence);

  unsigned int nextMmsi();
  static int   shipTypeToAisCode(const std::string& type);

 private: // Configuration
  bool         m_enabled;
  double       m_origin_lat;
  double       m_origin_lon;
  std::string  m_aivdm_host;
  int          m_aivdm_port;
  unsigned int m_mmsi_base;

  unsigned int m_random_count;
  double       m_random_radius;
  double       m_random_spd_min;
  double       m_random_spd_max;

  double       m_pos_interval;     // seconds between AIS message 1
  double       m_static_interval;  // seconds between AIS message 5

 private: // State
  std::vector<Contact> m_contacts;
  unsigned int         m_mmsi_next;
  double               m_ownship_x;
  double               m_ownship_y;
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
};

#endif
