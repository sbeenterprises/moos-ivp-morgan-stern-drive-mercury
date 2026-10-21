#--------------------------------------------------------
# The CMakeLists.txt for:                       iSignalK
# Author(s):                              D Lima
#--------------------------------------------------------

SET(SRC
  SignalK.cpp
  SignalK_Info.cpp
  main.cpp
)

# Find MOOSGeodesy before linking: newer MOOS-IvP provides it as an
# imported target, so ${MOOSGeodesy_LIBRARIES} must be set before
# TARGET_LINK_LIBRARIES (it was empty when found afterwards).
find_package(MOOSGeodesy)
include_directories(${MOOSGeodesy_INCLUDE_DIRS})

ADD_EXECUTABLE(iSignalK ${SRC})

find_package(CURL REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)

include_directories(${CURL_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})


TARGET_LINK_LIBRARIES(iSignalK
   ${MOOS_LIBRARIES}
   ${CURL_LIBRARIES}
   ${JSONCPP_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   jsoncpp_lib
   apputil
   mbutil
   m
   pthread)
