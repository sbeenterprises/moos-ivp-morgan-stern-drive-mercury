/************************************************************/
/*    NAME: DL                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: CAN.h                                          */
/*    DATE: December 29th, 1963                             */
/*          October 2026: CAN frames now come from a per-   */
/*          boat profile (profiles/NAME.prf, lib_canprofile)   */
/************************************************************/

#ifndef CAN_HEADER
#define CAN_HEADER

#include <map>
#include <set>
#include <string>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "CanProfile.h"
#include "DriveCore.h"

class CAN : public AppCastingMOOSApp
{
 public:
   CAN();
   ~CAN();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   bool initCAN();
   void closeCAN();
   void readFrames(double now);
   void sendDue(const canprofile::DriveCommand& cmd, double now);
   void sendNeutralOnce(double now);
   bool sendFrame(const canprofile::Frame& frame);
   bool msgEnabled(const canprofile::TxMsg& msg) const;
   std::string msgState(size_t ix, double now) const;
   void handleDemand(const CMOOSMsg& msg, bool is_rudder);
   void handleTrim(const CMOOSMsg& msg);
   void sendTrimIdle(double now);
   void postFeedback(const canprofile::RxValue& v, double now);
   void checkRudderFeedback(const canprofile::DriveCommand& cmd, double now);
   void postStatus(const canprofile::DriveCommand& cmd, double now);

 private: // Configuration variables
   std::string m_profile_file;
   std::string m_can_interface;
   std::set<std::string> m_manual_sources;
   std::set<std::string> m_autonomy_sources;
   std::set<std::string> m_trim_sources;
   double      m_link_stale_secs;

 private: // State variables
   canprofile::Profile   m_profile;
   canprofile::DriveCore m_core;
   bool        m_profile_ok;

   int         m_can_socket;
   double      m_last_init_attempt;
   double      m_start_time;
   std::string m_can_error;

   bool        m_can_send;
   bool        m_ui_link_ok;
   double      m_ui_link_time;

   std::vector<double> m_last_sent;      // per tx message
   std::vector<double> m_foreign_seen;   // per tx message: last time another node sent its ID
   std::vector<unsigned int> m_sent_count;
   unsigned int m_tx_errors;
   unsigned int m_rx_frames;

   double      m_last_iterate;
   double      m_tick;

   unsigned int m_rejected;
   std::string  m_last_rejected;
   std::string  m_trim_note;

   std::map<std::string, double>      m_fb_dval;
   std::map<std::string, std::string> m_fb_sval;
   std::map<std::string, double>      m_fb_time;
   std::map<std::string, double>      m_fb_posted;

   double      m_mismatch_since;
   bool        m_mismatch;

   canprofile::DriveCommand m_last_cmd;
   double      m_last_status_post;
   std::string m_last_status;
   std::string m_last_conflict;
};

#endif
