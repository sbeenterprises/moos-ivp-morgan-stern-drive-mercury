/************************************************************/
/*    NAME: D Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: WebSocket.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef WebSocket_HEADER
#define WebSocket_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXUserAgent.h>
#include <iostream>
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"


class WebSocket : public AppCastingMOOSApp
{
 public:
   WebSocket();
   ~WebSocket();
   std::string msg_recebida;
   
   // Add a method to send messages
   void SendMessage(const std::string& message, bool binary = false);
   
   // Add a method to set the current web socket connection
   void SetWebSocket(ix::WebSocket* ws);
   
   // Add wrapper methods for the protected MOOSApp Notify methods
   bool NotifyMOOS(const std::string& key, const std::string& value);
   bool NotifyMOOS(const std::string& key, double value);
   bool NotifyMOOS(const std::string& key, bool value);
   
   // Method to check if a client is already connected
   bool IsClientConnected() const;
   

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
  ix::WebSocket* m_webSocket; // Store the current web socket
  bool m_clientConnected; // Track if a client is already connected
  CMOOSGeodesy m_geodesy;
  
};

#endif 
