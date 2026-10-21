#--------------------------------------------------------
# The CMakeLists.txt for:                       iWebSocket
# Author(s):                              D Lima
#--------------------------------------------------------

SET(SRC
  WebSocket.cpp
  WebSocket_Info.cpp
  main.cpp
)

# Find MOOSGeodesy before linking: newer MOOS-IvP provides it as an
# imported target, so ${MOOSGeodesy_LIBRARIES} must be set before
# TARGET_LINK_LIBRARIES (it was empty when found afterwards).
find_package(MOOSGeodesy)
include_directories(${MOOSGeodesy_INCLUDE_DIRS})

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
   regiongeom
   apputil
   mbutil
   m
   pthread
   ixwebsocket
   OpenSSL::SSL
   OpenSSL::Crypto
   ${ZLIB_LIBRARIES})
