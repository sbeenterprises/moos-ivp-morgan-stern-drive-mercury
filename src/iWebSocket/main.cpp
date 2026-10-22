/************************************************************/
/*    NAME: D Lima                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: main.cpp, Cambridge MA                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <string>
#include "MBUtils.h"
#include "ColorParse.h"
#include "WebSocket.h"
#include "WebSocket_Info.h"
#include <ixwebsocket/IXWebSocketServer.h>
#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <ixwebsocket/IXUserAgent.h>
#include <iostream>

using namespace std;

// We'll use the isNumber function from MBUtils instead of defining our own

int port = 8000;
std::string host("0.0.0.0"); // If you need this server to be accessible on a different machine, use "0.0.0.0"

// Server ping interval (seconds). Browsers answer pings automatically, even
// with the UI tab idle or in the background; each pong refreshes the SS UI
// link time used by the lost-link failsafe (UI_LINK_OK), so keep this well
// below link_timeout. It must be given to the server constructor: IXWebSocket
// (11.4.x) applies it when it sets up each connection, so calling
// setPingInterval() in the Open callback had no effect and the server never
// pinged. The link then only stayed up while the UI itself sent something,
// and with the UI idle it was up 3 s out of every 30 s (its keep-alive).
const int PING_INTERVAL_SECONDS = 1;

ix::WebSocketServer server(port, host,
                           ix::SocketServer::kDefaultTcpBacklog,
                           ix::SocketServer::kDefaultMaxConnections,
                           ix::WebSocketServer::kDefaultHandShakeTimeoutSecs,
                           ix::SocketServer::kDefaultAddressFamily,
                           PING_INTERVAL_SECONDS);


int main(int argc, char *argv[])
{
    
    //Instance of MOOS-IvP object  

    WebSocket WebSocket;
    
    //Start of WebSocket server to make the interface with SignalK

    server.setOnClientMessageCallback([&](std::shared_ptr<ix::ConnectionState> connectionState, ix::WebSocket & webSocket, const ix::WebSocketMessagePtr & msg) {
      // The ConnectionState object contains information about the connection,
      // at this point only the client ip address and the port.
      std::cout << "Remote ip: " << connectionState->getRemoteIp() << std::endl;

      if (msg->type == ix::WebSocketMessageType::Open)
      {
          std::cout << "New connection" << std::endl;

          // A connection state object is available, and has a default id
          // You can subclass ConnectionState and pass an alternate factory
          // to override it. It is useful if you want to store custom
          // attributes per connection (authenticated bool flag, attributes, etc...)
          std::cout << "id: " << connectionState->getId() << std::endl;

          // The uri the client did connect to.
          std::cout << "Uri: " << msg->openInfo.uri << std::endl;

          std::cout << "Headers:" << std::endl;
          for (auto it : msg->openInfo.headers)
          {
              // Cookies and Authorization carry login tokens: never print them
              std::string name = tolower(it.first);
              bool secret = (name == "cookie" || name == "authorization");
              std::cout << "\t" << it.first << ": " << (secret ? "<redacted>" : it.second) << std::endl;
          }

          // command_auth = signalk: find out who this is before the client
          // is counted for the link. The browser sends its Signal K login
          // cookie with the upgrade request (same host, any port).
          if (WebSocket.CommandAuthOn()) {
              auto cookie = msg->openInfo.headers.find("Cookie");
              std::string token = (cookie == msg->openInfo.headers.end()) ? "" :
                                  WebSocket::TokenFromCookies(cookie->second);
              WebSocket.AuthorizeClient(&webSocket, token);
          }
          else
              webSocket.send("AUTH_STATUS=level=open#mode=off");

          // Add the new WebSocket connection to the list
          WebSocket.AddWebSocket(&webSocket);

          // Bring a new client up to date with the operator's UI
          std::string ui_state = WebSocket.LatestUiStateMessage();
          if (!ui_state.empty())
              webSocket.send(ui_state);
          std::cout << "Total connected clients: " << WebSocket.GetConnectedClientCount() << std::endl;
          
          // Pings are set on the server (PING_INTERVAL_SECONDS above)

      } 
      else if (msg->type == ix::WebSocketMessageType::Message)
      {
          // For an echo server, we just send back to the client whatever was received by the server
          // All connected clients are available in an std::set. See the broadcast cpp example.
          // Second parameter tells whether we are sending the message in binary or text mode.
          // Here we send it in the same mode as it was received.
          bool is_auth = (msg->str.compare(0, 5, "AUTH=") == 0);
          // UI_STATE arrives every time the operator changes something: not echoed
          if (msg->str.compare(0, 9, "UI_STATE=") != 0)
              std::cout << "Received: " << (is_auth ? "AUTH=<redacted>" : msg->str) << std::endl;

          // AUTH=<token>: the UI (re)sends its Signal K token after a login,
          // logout or reconnect. Never queued or posted.
          if (is_auth) {
              if (WebSocket.CommandAuthOn())
                  WebSocket.AuthorizeClient(&webSocket, msg->str.substr(5));
              else
                  webSocket.send("AUTH_STATUS=level=open#mode=off");
              WebSocket.UpdateConnectionTime(&webSocket);
              return;
          }

          // Update connection timestamp on message receive
          WebSocket.UpdateConnectionTime(&webSocket);

          // Read-only, anonymous or unverified users may watch, not command.
          // The UI's keep-alive is dropped quietly, anything else is refused.
          if (!WebSocket.ClientMayCommand(&webSocket)) {
              if (msg->str.compare(0, 15, "HEARTBEAT_PING=") != 0)
                  WebSocket.RejectCommand(&webSocket, msg->str);
              return;
          }
          
          //Adiciono o valor de msg a fila lida pelo Iterate()
          WebSocket.QueueMessage(msg -> str);

          // Parse the incoming message to publish to MOOS
          std::string message = msg->str;
          size_t equalsPos = message.find('=');
          
          if (equalsPos != std::string::npos) {
              std::string key = message.substr(0, equalsPos);
              std::string value = message.substr(equalsPos + 1);
              
              // Handle the message based on key-value
              if (key == "CONTROLE_MANUAL") {
                WebSocket.NotifyMOOS("DEPLOY", true);
              }
                  
              } 
          

          //webSocket.send(msg->str, msg->binary); //Faz o envio dessas msgs pelo websocket

      } 
      else if (msg->type == ix::WebSocketMessageType::Close)
      {
          std::cout << "Client disconnected" << std::endl;
          std::cout << "Close code: " << msg->closeInfo.code << std::endl;
          std::cout << "Close reason: " << msg->closeInfo.reason << std::endl;
          // Remove the WebSocket from the list when client disconnects
          WebSocket.RemoveWebSocket(&webSocket);
          std::cout << "Total connected clients: " << WebSocket.GetConnectedClientCount() << std::endl;
      }
      else if (msg->type == ix::WebSocketMessageType::Error)
      {
          std::cerr << "WebSocket Error: " << msg->errorInfo.reason << std::endl;
          std::cerr << "Error code: " << msg->errorInfo.http_status << std::endl;
          // Connection error - remove from list
          WebSocket.RemoveWebSocket(&webSocket);
      }
      else if (msg->type == ix::WebSocketMessageType::Ping)
      {
          std::cout << "Received ping from client" << std::endl;
          // Pong is sent automatically by the library
          WebSocket.UpdateConnectionTime(&webSocket);
      }
      else if (msg->type == ix::WebSocketMessageType::Pong)
      {
          // (every second per client: no console print)
          WebSocket.UpdateConnectionTime(&webSocket);
      }
  });

  string mission_file;
  string run_command = argv[0];

  for(int i=1; i<argc; i++) {
    string argi = argv[i];
    if((argi=="-v") || (argi=="--version") || (argi=="-version"))
      showReleaseInfoAndExit();
    else if((argi=="-e") || (argi=="--example") || (argi=="-example"))
      showExampleConfigAndExit();
    else if((argi == "-h") || (argi == "--help") || (argi=="-help"))
      showHelpAndExit();
    else if((argi == "-i") || (argi == "--interface"))
      showInterfaceAndExit();
    else if(strEnds(argi, ".moos") || strEnds(argi, ".moos++"))
      mission_file = argv[i];
    else if(strBegins(argi, "--alias="))
      run_command = argi.substr(8);
    else if(i==2)
      run_command = argi;
  }
  
  if(mission_file == "")
    showHelpAndExit();

  cout << termColor("green");
  cout << "iWebSocket launching as " << run_command << endl;
  cout << termColor() << endl;

  // The server accepts clients before the MOOS app has read its config
  // (that happens in Run), so command_auth must be known before it starts:
  // a client let in early would otherwise never be authorised.
  WebSocket.ReadAuthConfig(mission_file, run_command);

  auto res = server.listen();
  if (!res.first)
  {
      // Error handling
      return 1;
  }

  // Per message deflate connection is enabled by default. It can be disabled
  // which might be helpful when running on low power devices such as a Rasbery Pi
  server.disablePerMessageDeflate();

  // Run the server in the background. Server can be stoped by calling server.stop()
  server.start();

  WebSocket.Run(run_command.c_str(), mission_file.c_str());
  
  return(0);
}

