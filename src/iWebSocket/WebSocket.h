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
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"


class WebSocket : public AppCastingMOOSApp
{
 public:
   WebSocket();
   ~WebSocket();
   std::string msg_recebida;
   
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
  std::vector<ix::WebSocket*> m_webSockets; // Store multiple web socket connections
  std::map<ix::WebSocket*, double> m_lastMessageTime; // Track last message time for each connection
  CMOOSGeodesy m_geodesy;
  
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
  void handleTrim(const std::string& value);
  void handleStationKeepWaypoint(const std::string& value);
  void handleHeartbeatPing(const std::string& value);

};

#endif 
