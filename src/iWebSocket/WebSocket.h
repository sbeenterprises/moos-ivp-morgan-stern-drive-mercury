/************************************************************/
/*    NAME: D Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: WebSocket.h                                          */
/*    DATE: December 29th, 1963                             */
/***********************************************************/

#ifndef WebSocket_HEADER
#define WebSocket_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXUserAgent.h>
#include <iostream>
#include <map>
#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <set>
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "RegionGeom.h"
#include "ControlLease.h"


class WebSocket : public AppCastingMOOSApp
{
 public:
   WebSocket();
   ~WebSocket();
   std::string msg_recebida; // last message dispatched, shown in the appcast report

   // Queue a message received from the UI. Called from the websocket thread;
   // the messages are dispatched later from Iterate() on the MOOS thread.
   // This must be a queue and not a single slot: the UI sends bursts (a mode
   // switch immediately followed by its gains) and everything but the last
   // message of a burst would otherwise be overwritten before it is read.
   // ws: the sender (for the control generation and refusals); all_stop:
   // an ALL_STOP from an operator who need not hold control
   void QueueMessage(ix::WebSocket* ws, const std::string& message, bool all_stop = false);

   // Add a method to send messages
   void SendMessage(const std::string& message, bool binary = false);
   
   // Add methods to manage multiple web socket connections
   void AddWebSocket(ix::WebSocket* ws);
   void RemoveWebSocket(ix::WebSocket* ws);

   // Add wrapper methods for the protected MOOSApp Notify methods
   bool NotifyMOOS(const std::string& key, const std::string& value);
   bool NotifyMOOS(const std::string& key, double value);
   bool NotifyMOOS(const std::string& key, bool value);
   bool OnDisconnectFromServer();

   // Method to check if any clients are connected
   bool HasConnectedClients() const;

   // Method to get number of connected clients
   size_t GetConnectedClientCount() const;
   
   // Connection monitoring methods
   void UpdateConnectionTime(ix::WebSocket* ws);
   void CheckStaleConnections();

   // Command authorisation (command_auth = signalk). Each connection gets
   // the user's Signal K permission (readonly / readwrite / admin) from the
   // Signal K server; only readwrite and admin may command the boat. The
   // token comes from the JAUTHENTICATION / sktoken cookie on connect or
   // from an AUTH=<token> message. Called from the websocket threads.
   bool CommandAuthOn() const { return m_command_auth; }
   void ReadAuthConfig(const std::string& mission_file, const std::string& app_name);
   static std::string TokenFromCookies(const std::string& cookie_header);
   // value: <token>[#session=<tab id>][#logout]; an empty token falls back
   // on the login cookie the connection arrived with, unless #logout
   void AuthorizeClient(ix::WebSocket* ws, const std::string& value);
   void SetCookieToken(ix::WebSocket* ws, const std::string& token);
   void SetClientIp(ix::WebSocket* ws, const std::string& ip);
   bool ClientMayCommand(ix::WebSocket* ws);
   void RejectCommand(ix::WebSocket* ws, const std::string& message);

   // Exclusive control (exclusive_control = true, needs command_auth). One
   // session holds the control lease (ControlLease); only it may command,
   // its traffic alone keeps UI_LINK_OK up. CTRL=take|release|request|
   // accept|deny|cancel|force#reason=..; ALL_STOP from any operator.
   bool ExclusiveOn() const { return m_exclusive; }
   void HandleControl(ix::WebSocket* ws, const std::string& value);
   bool ClientMayAllStop(ix::WebSocket* ws);

   // Operator UI state (UI_STATE=<json>): what the commanding UI has selected
   // or drawn that the boat does not hold itself (mission route, speed, base,
   // corridor, station-keep point...). Relayed to every client so read-only
   // viewers see it as it happens; a client that connects gets the latest.
   std::string LatestUiStateMessage();


 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
  // Messages received from the UI, waiting to be dispatched by Iterate().
  // Written from the websocket thread, drained from the MOOS thread.
  struct Inbound {
    std::string message;
    ix::WebSocket* ws;   // sender; may have closed by the time it is handled
    unsigned gen;        // control generation when it arrived
    std::string user;
    bool all_stop;
  };
  std::deque<Inbound> m_inbox;
  std::mutex m_inbox_mutex;

  std::vector<ix::WebSocket*> m_webSockets; // Store multiple web socket connections
  std::map<ix::WebSocket*, double> m_lastMessageTime; // Track last message time for each connection
  CMOOSGeodesy m_geodesy;     // initialised once from LatOrigin/LongOrigin
  bool   m_origin_ok;          // false: mission has no LatOrigin/LongOrigin
  double m_lat_origin;
  double m_lon_origin;
  bool   originReady(const std::string& what);
  
  // Connection monitoring
  double m_connection_timeout; // Timeout in seconds for stale connections
  double m_last_connection_check; // Last time we checked for stale connections
  
  // Link health for the lost-link failsafe in iCAN. Any message, ping
  // or pong from a client counts; browsers answer the server's 1 Hz
  // WebSocket ping automatically, so SS UI needs no change for this.
  std::atomic<double> m_last_link_rx;   // written from the websocket thread
  double m_link_timeout;                // seconds of silence = link lost
  bool   m_link_ok_posted;
  double m_last_link_post;
  // lost_link_autonomy = stop: a lost link ends the autonomous mode
  // (as AUTONOMOUS_STOP does), so the boat stays stopped when it returns.
  bool   m_lost_link_stop;

  // Operating region. The UI's polygon is the HALT line; the boat works
  // inside it shrunk by region_margin (lib_regiongeom), handed to
  // BHV_OpRegionV26 through REGION_POLY with breach_dist = region_margin.
  double m_region_margin;        // metres, 0 = the drawn line is the working boundary
  double m_region_halt_lead;     // halt this far before the drawn line (coasting room)
  double m_region_recover_speed; // helm units to steer back in; <= 0 = alpha.bhv's
  bool   m_nav_xy_known;
  double m_nav_x, m_nav_y;
  std::string m_region_state;    // V26_STATE while REGION is on, else "off"
  std::string m_region_core;     // working boundary, "lat,lon:lat,lon:..."
  std::string m_region_msg;      // last refusal / note for the UI
  std::string m_region_spec;     // last REGION_POLY posted
  std::string m_region_updates;  // last REGION_UPDATES posted: re-posted when the
                                 // helm takes control (it drops updates in manual)
  double m_region_repost_at;     // next REGION_POLY re-post (every 2 s while REGION is on)
  bool   applyRegionPolygon(const std::string& latlon_pts);
  // Autonomy may allow a longer outage than manual: UI_LINK_OK_AUTO uses
  // autonomy_link_timeout (default: link_timeout).
  double m_auto_link_timeout;
  bool   m_auto_link_ok_posted;
  void   stopAutonomyOnLostLink();
  // Ends a running mission / station-keep / follow as AUTONOMOUS_STOP does
  // and posts event_var = the modes it ended.
  void   stopAutonomy(const std::string& why, const std::string& event_var);

  // Navigation integrity (pNavGuard). NAV_GUARD_STOP ends autonomy;
  // while NAV_STATUS is fault, starts are refused. nav_guard_required:
  // also refuse them when pNavGuard has not been heard for 5 s.
  bool   m_nav_guard_required;
  std::string m_nav_status;      // ok | warn | fault, "" = never heard
  std::string m_nav_status_msg;
  double m_nav_status_time;
  std::string m_nav_stop_msg;    // why the last nav stop happened (UI)
  std::string m_nav_refused;     // last start refused for nav (UI)
  bool   navReady(const std::string& what);

  // iCAN status passed through to SS UI (CAN_* variables)
  std::set<std::string> m_can_vars;
  std::map<std::string, std::string> m_can_str;
  std::map<std::string, double> m_can_num;

  // Command authorisation (see CommandAuthOn). With it on, only clients
  // allowed to command keep the link up (UI_LINK_OK*): a read-only viewer
  // left open must not hold off the lost-link failsafe.
  struct ClientAuth {
    std::string level;    // Signal K: readonly | readwrite | admin | open | none | unavailable
    std::string user;
    std::string role;     // viewer | planner | operator | supervisor
    std::string session;  // the browser tab's id (AUTH=<token>#session=<id>)
    std::string ip;
    ControlLease::Session sess() const { return ControlLease::Session{session, user, role}; }
  };
  bool        m_command_auth;    // command_auth = signalk
  std::string m_signalk_http;    // signalk_http = host:port of Signal K (plain HTTP)
  std::map<ix::WebSocket*, ClientAuth> m_client_auth;
  std::map<ix::WebSocket*, std::string> m_cookie_token;   // login cookie at connect
  std::map<ix::WebSocket*, std::string> m_client_ip;
  // Audit trail: every login, logout, disconnect, control change, ALL STOP
  // and refused command, stamped when it happens ("kind|UTC time|text"),
  // posted to MOOS (UI_AUTH_EVENT / CONTROL_EVENT) and appended to
  // <audit_dir>/c2_audit_<UTC date>.log. Callers hold m_auth_mutex.
  std::string m_audit_dir;
  bool        m_audit_warned;
  std::string m_dispatch_user;   // sender of the command being handled (MOOS thread)
  void pushEvent(const std::string& kind, const std::string& text);
  void drainLease();
  void writeAudit(const std::string& stamp, const std::string& kind, const std::string& text);
  static std::string utcStamp();
  static std::string shortId(const std::string& session);
  std::deque<std::string> m_auth_events;  // filled by websocket threads, posted by Iterate
  unsigned    m_cmds_rejected;
  std::mutex  m_auth_mutex;      // guards the four above
  static bool levelMayCommand(const std::string& level);
  bool        setAuthParam(const std::string& param, const std::string& value);
  std::string roleFor(const std::string& user, const std::string& level);
  void        sendControlStatus();     // per client; needs m_auth_mutex free

  bool         m_exclusive;            // exclusive_control
  std::string  m_roles_file;           // roles_file: Signal K plugin config JSON
  ControlLease m_lease;                // guarded by m_auth_mutex
  bool         m_ctrl_dirty;           // send CONTROL_STATUS soon
  double       m_ctrl_sent_at;
  std::atomic<bool> m_boat_idle;       // no mission / station keep / follow / manual
  std::atomic<bool> m_moos_up;
  bool         m_moos_connected_once;
  bool         m_manual_on;            // MOOS_MANUAL_OVERRIDE
  bool         m_all_stop;             // ALL STOP latched until the holder resets it
  std::string  m_all_stop_by;
  void handleAllStop(const std::string& user);
  bool allStopBlocks(const std::string& key, const std::string& value) const;
  bool        clientCountsForLink(ix::WebSocket* ws);
  bool        hasLinkClients();

  // Heartbeat to keep connection alive
  double m_heartbeat_interval; // Interval in seconds to send heartbeat messages
  double m_last_heartbeat_time; // Last time we sent a heartbeat
  
  // State machine for message handling
  typedef void (WebSocket::*MessageHandler)(const std::string& value);
  std::map<std::string, MessageHandler> m_messageHandlers;
  void initializeMessageHandlers();
  
  // Message handler methods
  void handleTrailFollow(const std::string& value);
  void handleControleManual(const std::string& value);
  void handleAutonomousControl(const std::string& value);
  void handleDesiredHeading(const std::string& value);
  void handleDesiredSpeed(const std::string& value);
  void handleDesiredRudder(const std::string& value);
  void handleDesiredThrust(const std::string& value);
  void handleDesiredGear(const std::string& value);
  void handleAutonomousWaypoints(const std::string& value);
  void handleAutonomousStart(const std::string& value);
  void handleAutonomousStop(const std::string& value);
  void handleAutonomousReturnToBase(const std::string& value);
  void handleReturnUpdate(const std::string& value);
  void handleStopCan(const std::string& value);
  void handleConstantHeading(const std::string& value);
  void handleSetpointHeading(const std::string& value);
  void handleHeadingKp(const std::string& value);
  void handleHeadingKi(const std::string& value);
  void handleHeadingKd(const std::string& value);
  void handleSpeedKp(const std::string& value);
  void handleSpeedKi(const std::string& value);
  void handleSpeedKd(const std::string& value);
  void handleServiceMode(const std::string& value);
  void handleColavTest(const std::string& value);
  void handlePutTarget(const std::string& value);
  void handleClearTargets(const std::string& value);
  void handleColavDist(const std::string& value);
  void applyColavDist();
  void handleTrim(const std::string& value);
  void handleStationKeepWaypoint(const std::string& value);
  void handleHeartbeatPing(const std::string& value);
  void handleRepeat(const std::string& value);
  void handleBaseCoords(const std::string& value);
  void handleCanSend(const std::string& value);
  void handleRegionUpdates(const std::string& value);
  void handleRegion(const std::string& value);
  bool handleRegionUpdatesOk(const std::string& value);
  void handleCorridor(const std::string& value);
  void handleLoopStart(const std::string& value);
  void handleUiState(const std::string& value);
  void handleAllStopReset(const std::string& value);
  std::string m_ui_state;        // latest UI_STATE payload, guarded by m_ui_state_mutex
  std::mutex  m_ui_state_mutex;

};

#endif 
