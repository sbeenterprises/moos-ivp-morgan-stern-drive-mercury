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
#include <ixwebsocket/IXWebSocketServer.h>
#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <deque>
#include <mutex>
#include <atomic>
#include <jsoncpp/json/json.h>
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"


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
   void QueueMessage(const std::string& message);

   // Add a method to send messages
   void SendMessage(const std::string& message, bool binary = false);
   
   // Add methods to manage multiple web socket connections
   void AddWebSocket(ix::WebSocket* ws);
   void RemoveWebSocket(ix::WebSocket* ws);

   // Add wrapper methods for the protected MOOSApp Notify methods
   bool NotifyMOOS(const std::string& key, const std::string& value);
   bool NotifyMOOS(const std::string& key, double value);
   bool NotifyMOOS(const std::string& key, bool value);

   // Method to check if any clients are connected
   bool HasConnectedClients() const;

   // Method to get number of connected clients
   size_t GetConnectedClientCount() const;
   
   // Connection monitoring methods
   void UpdateConnectionTime(ix::WebSocket* ws);
   void CheckStaleConnections();

   // Any frame from a UI client - message, ping, pong or a new connection -
   // proves the link to the shore is up. Feeds the comms-loss watchdog.
   void NoteUiFrame();

   // The server is created in main(), before the MOOS loop starts. Messages
   // are broadcast through its own client list, whose shared pointers keep a
   // connection alive while it is being written to - a closing client is
   // destroyed by its own thread as soon as its close callback returns.
   void SetServer(ix::WebSocketServer* server, int port);


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
  // Comms-loss failsafe. The UI runs ashore and the link to it can drop; when
  // nothing has been heard from any UI for comms_loss_timeout seconds the boat
  // returns to the base on its own (RETURN in pHelmIvP), and holds there.
  bool        m_comms_loss_return;         // false = report the loss only
  double      m_comms_loss_timeout;        // seconds without a frame from any UI
  double      m_comms_loss_speed;          // return speed, in the helm's speed units (knots on the boat)
  bool        m_comms_loss_always;         // also with the helm in PARK and the boat idle
  double      m_underway_speed;            // NAV_SPEED above which the boat is underway

  // The base the boat returns to. Starts from base_lat/base_lon, then follows
  // BASE_COORDS from the UI. base_file, when set, keeps the last base across
  // restarts.
  double      m_base_lat;
  double      m_base_lon;
  std::string m_base_file;
  double      m_lat_origin;
  double      m_lon_origin;

 private: // State variables
  // Messages received from the UI, waiting to be dispatched by Iterate().
  // Written from the websocket thread, drained from the MOOS thread.
  std::deque<std::string> m_inbox;
  std::mutex m_inbox_mutex;

  // Connected clients and when each was last heard from. Added and removed by
  // the websocket threads, read by the MOOS thread - always under the mutex.
  std::vector<ix::WebSocket*> m_webSockets; // Store multiple web socket connections
  std::map<ix::WebSocket*, double> m_lastMessageTime; // Track last message time for each connection
  mutable std::mutex m_clients_mutex;
  std::vector<std::string> m_client_events;  // appcast events queued by the websocket threads
  ix::WebSocketServer* m_server;
  int m_port;
  CMOOSGeodesy m_geodesy;

  // Last frame of any kind (message, ping, pong) from any UI client, MOOS
  // time. Written by the websocket threads, read by the watchdog.
  std::atomic<double> m_last_ui_rx;
  std::atomic<double> m_last_ui_rx_real;   // the same, wall clock (no time warp), for display
  std::atomic<bool>   m_ui_seen;           // the watchdog arms on the first frame

  bool        m_comms_lost;
  bool        m_comms_loss_handled;        // the current loss has been acted on
  double      m_comms_lost_at;             // last frame before the loss, wall clock
  double      m_last_loss_duration;        // wall clock seconds, of the last loss that ended
  unsigned int m_comms_loss_count;
  std::string m_comms_loss_action;         // what the watchdog did about the last loss

  // Mirrors of RETURN / RETURN_HOLD. Set here as well as from the mail, so a
  // command handled in the same tick as the return sees it.
  bool        m_return_active;
  bool        m_hold_active;
  bool        m_return_by_failsafe;        // engaged by the watchdog, not an operator
  std::string m_return_reason;
  double      m_return_idle_since;         // RETURN and RETURN_HOLD both off since
  std::string m_stationing;                // STATIONING, for the underway test

  double      m_base_x;                    // base in the local grid
  double      m_base_y;
  std::string m_base_source;               // config, file or ui
  bool        m_base_announced;            // posted to the helm since startup

  // Manual rudder/thrust that ended a return, held back for a moment so it
  // lands after planchaPID's own zero on letting go of the actuators.
  std::map<std::string, double> m_deferred_manual;
  double      m_deferred_manual_at;

  // Collision alerts: the contacts the helm is steering around, sent to the
  // UI's Alert List as COLAV_ALERTS. See updateColavAlerts().
  struct ColavAlert {
    bool   active;                         // the helm is avoiding it right now
    double since;                          // start of this avoidance, MOOS time
    double last_active;                    // last seen active, for the hold
    std::string mode;                      // COLREGS mode last classified in this avoidance
    ColavAlert() : active(false), since(0), last_active(0) {}
  };
  std::map<std::string, ColavAlert>  m_colav_alerts;   // by contact name
  std::set<std::string>              m_helm_avoiding;  // contacts of the active avdcol_ instances
  std::map<std::string, std::string> m_colav_mode;     // latest COLREGS mode, by contact
  std::string m_colav_alerts_json;
  double      m_colav_alerts_built;
  bool        m_colav_alerts_dirty;                    // rebuild on the next tick
  double      m_colav_alert_hold;                      // s an alert stays up once the avoidance ends
  double      m_nav_speed_to_mps;                      // NAV_SPEED to m/s (knots on the boat)
  
  // Connection monitoring
  double m_connection_timeout; // Timeout in seconds for stale connections
  double m_last_connection_check; // Last time we checked for stale connections
  
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
  void handleAisRadius(const std::string& value);
  void applyAisLimit(bool force);
  void handleTrim(const std::string& value);
  void handleStationKeepWaypoint(const std::string& value);
  void handleHeartbeatPing(const std::string& value);
  void handleRepeat(const std::string& value);
  void handleBaseCoords(const std::string& value);
  void handleCanSend(const std::string& value);
  void handleRegionUpdates(const std::string& value);
  void handleRegion(const std::string& value);

  // Comms-loss failsafe
  void checkCommsLoss();
  bool isUnderway() const;
  std::string stayReason() const;
  void engageReturn(const std::string& reason);
  bool cancelReturn(const std::string& reason);
  void postManual(const std::string& var, double value, bool cancelled);
  bool setBase(double lat, double lon, const std::string& source);
  bool postBaseToHelm();
  bool loadBaseFile();
  void saveBaseFile() const;
  Json::Value commsLossJson() const;

  // Collision alerts
  void updateColavAlerts();

};

#endif 
