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
ix::WebSocketServer server(port, host);

// Configure server heartbeat (ping interval in seconds)
const int PING_INTERVAL_SECONDS = 30;


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
              std::cout << "\t" << it.first << ": " << it.second << std::endl;
          }

          // Add the new WebSocket connection to the list
          WebSocket.AddWebSocket(&webSocket);
          std::cout << "Total connected clients: " << WebSocket.GetConnectedClientCount() << std::endl;
          
          // Enable heartbeat for this connection
          webSocket.setPingInterval(PING_INTERVAL_SECONDS);

      } 
      else if (msg->type == ix::WebSocketMessageType::Message)
      {
          // For an echo server, we just send back to the client whatever was received by the server
          // All connected clients are available in an std::set. See the broadcast cpp example.
          // Second parameter tells whether we are sending the message in binary or text mode.
          // Here we send it in the same mode as it was received.
          std::cout << "Received: " << msg->str << std::endl;
          
          // Update connection timestamp on message receive
          WebSocket.UpdateConnectionTime(&webSocket);
          
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
          std::cout << "Received pong from client" << std::endl;
          WebSocket.UpdateConnectionTime(&webSocket);
      }
  });

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

  // Block until server.stop() is called.
  //server.wait();
  // Or you can use server.stop() to stop the server in another thread.  
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

  WebSocket.Run(run_command.c_str(), mission_file.c_str());
  
  return(0);
}

