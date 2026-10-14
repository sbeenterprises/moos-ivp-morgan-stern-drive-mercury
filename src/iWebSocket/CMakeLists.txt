#--------------------------------------------------------
# The CMakeLists.txt for:                       iWebSocket
# Author(s):                              D Lima
#--------------------------------------------------------

SET(SRC
  WebSocket.cpp
  WebSocket_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iWebSocket ${SRC})

find_package(OpenSSL REQUIRED)
find_package(ZLIB REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)

include_directories(${JSONCPP_INCLUDE_DIRS})


TARGET_LINK_LIBRARIES(iWebSocket
   ${MOOS_LIBRARIES}
   ${JSONCPP_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   jsoncpp_lib
   geometry
   apputil
   mbutil
   m
   pthread
   ixwebsocket
   OpenSSL::SSL
   OpenSSL::Crypto
   ${ZLIB_LIBRARIES})

#Find MOOSGeodesy
find_package(MOOSGeodesy)
include_directories(${MOOSGeodesy_INCLUDE_DIRS} ${MOOSGeodesy_LIBRARIES})
