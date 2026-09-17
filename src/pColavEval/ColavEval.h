/************************************************************/
/*    NAME: DL                                              */
/*    FILE: ColavEval.h                                     */
/*    DATE: 2026                                            */
/*                                                          */
/*  Measures what BHV_AvdColregsV22 actually did.           */
/*                                                          */
/*  One record per encounter, keyed by contact plus a        */
/*  sequence number - the helm respawns a behaviour on the  */
/*  same contact if it completes and later re-alerts, and   */
/*  those are two different encounters.                     */
/*                                                          */
/*  Two sources, deliberately kept apart:                   */
/*    - the behaviour's own view, via the COLAV_BHV_* flags */
/*      configured on the avdcol_ template (mode, priority, */
/*      when it went active, when it saw its CPA);          */
/*    - ground truth, tracked here from NODE_REPORT and     */
/*      NAV_*, which is what the achieved CPA is taken from.*/
/*  The behaviour runs on an extrapolated contact position  */
/*  (extrapolate=true, decay=30,60), so its range is not    */
/*  the number to report as a result.                       */
/************************************************************/

#ifndef ColavEval_HEADER
#define ColavEval_HEADER

#include <string>
#include <vector>
#include <map>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class ColavEval : public AppCastingMOOSApp
{
 public:
  ColavEval();
  ~ColavEval() {}

 protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

 protected:
  /** Last known state of one contact, straight off NODE_REPORT. */
  struct CnState {
    double x, y, hdg, spd, utc;
    bool   valid;
    CnState() { x=0; y=0; hdg=0; spd=0; utc=0; valid=false; }
  };

  /** One pass of one contact, from behaviour spawn to behaviour death. */
  struct Encounter {
    std::string  cn;
    unsigned int seq;
    unsigned int mmsi;
    bool         is_test;      // MMSI inside the runtime-spawn block
    std::string  state;        // live | past_cpa | done

    // Timeline. -1 means "has not happened yet".
    double t_placed;           // TRAFFIC_SPAWNED, if this target was placed here
    double t_detect;           // COLAV_BHV_SPAWN - behaviour instance created
    double t_active;           // first COLAV_BHV_ACTIVE - first helm influence
    double t_cpa;              // COLAV_BHV_CPA - behaviour saw its own CPA
    double t_end;              // COLAV_BHV_END / life event death

    double rng_detect;
    double rng_active;

    // Predicted at detect vs what actually happened.
    double cpa_pred;
    double tcpa_pred;
    double min_range;
    double t_min;
    double x_min, y_min;

    // Live values, refreshed every iteration while the encounter is open.
    double range;
    double cpa_live;
    double tcpa_live;

    std::string mode;          // latest $[FULL_MODE]
    std::string mode_history;  // distinct modes, in order, ':'-free ',' joined
    double      apwt_max;

    double hdg_at_detect;
    // Deflection is measured from the heading at the moment the behaviour first
    // took the helm, not from detection. Before that the boat is steering the
    // route, and counting a leg turn as avoidance would flatter the result.
    double hdg_at_active;
    double deflection_max;     // max |heading - hdg_at_active|
    double des_deflection_max; // same against DESIRED_HEADING
    bool   deflection_valid;   // false once the waypoint index changes
    double wpt_at_active;

    double spd_at_detect;
    double spd_min;

    double dur_active;         // summed time in the ACTIVE state
    double active_since;       // -1 when not currently active
    unsigned int active_spans;

    // Relative bearings at CPA, for the ENCOUNTER_SUMMARY spec.
    double alpha, beta;

    double      ucd_cpa;       // independent oracle, -1 if uFldCollisionDetect is absent
    std::string end_reason;
    std::string post_mortem;
    std::string verdict;       // collision | near_miss | clear

    Encounter();
  };

  void   registerVariables();

  void   handleBhvSpawn(const std::string& sval);
  void   handleBhvActive(const std::string& sval);
  void   handleBhvInactive(const std::string& sval);
  void   handleBhvCpa(const std::string& sval);
  void   handleBhvEnd(const std::string& sval);
  void   handleBhvTrace(const std::string& sval);
  void   handleLifeEvent(const std::string& sval);
  void   handleNodeReport(const std::string& sval);
  void   handleTrafficSpawned(const std::string& sval);
  void   handleUcdReport(const std::string& sval);

  Encounter* openEncounter(const std::string& cn);
  Encounter* findOpen(const std::string& cn);
  void   updateLive(Encounter& e, double now);
  void   closeEncounter(Encounter& e, double now, const std::string& reason);
  void   noteMode(Encounter& e, const std::string& mode);

  void   postMetrics();
  std::string encounterJson(const Encounter& e) const;
  std::string encounterSpec(const Encounter& e) const;

 private: // Configuration
  double       m_collision_range;
  double       m_near_miss_range;
  double       m_report_interval;
  double       m_cpa_confirm;   // metres the range must open before calling CPA
  unsigned int m_max_records;
  unsigned int m_test_mmsi_low;
  unsigned int m_test_mmsi_high;

 private: // State
  std::vector<Encounter>          m_encounters;
  std::map<std::string, CnState>  m_contacts;
  std::map<std::string, double>   m_placed;   // vname -> TRAFFIC_SPAWNED utc

  double m_osx, m_osy, m_osh, m_osv, m_des_hdg;
  bool   m_os_known;
  // NAV_SPEED arrives on its own schedule. Without this the minimum-speed
  // metric latches the m_osv initialiser of 0 on the first iteration and never
  // recovers, so every encounter reports a min speed of zero.
  bool   m_osv_known;
  double m_wpt_index;
  bool   m_waypoint_active;
  bool   m_test_mode;

  double      m_last_post;
  std::string m_last_metrics;
  unsigned int m_seq_next;
};

#endif
