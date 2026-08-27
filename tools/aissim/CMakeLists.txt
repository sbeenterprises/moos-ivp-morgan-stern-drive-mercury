#--------------------------------------------------------
# The CMakeLists.txt for:                            aissim
# Author(s):                                   Douglas Lima
#
# A standalone tool, not a MOOS app: it never connects to a
# MOOSDB. It does link MOOSGeodesy so that its local grid is
# projected with exactly the same code pTrafficSim and
# iDivisorNMEA use - a different projection here would put
# contacts in a different place than MOOS thinks they are.
#
# The AIVDM encoder is shared with pTrafficSim rather than
# copied, so both halves of the AIS path stay in step.
#--------------------------------------------------------

SET(SRC
  main.cpp
  HullModel.cpp
  NmeaOut.cpp
  TrafficFleet.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/pTrafficSim/AisEncode.cpp
)

ADD_EXECUTABLE(aissim ${SRC})

find_package(OpenSSL REQUIRED)
find_package(ZLIB REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(MOOSGeodesy)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../src/pTrafficSim
  ${MOOSGeodesy_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS})

TARGET_LINK_LIBRARIES(aissim
  ${MOOSGeodesy_LIBRARIES}
  ${MOOS_LIBRARIES}
  jsoncpp_lib
  ixwebsocket
  OpenSSL::SSL
  OpenSSL::Crypto
  ${ZLIB_LIBRARIES}
  m
  pthread)
