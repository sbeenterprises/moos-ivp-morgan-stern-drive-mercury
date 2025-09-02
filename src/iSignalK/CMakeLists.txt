#--------------------------------------------------------
# The CMakeLists.txt for:                       iSignalK
# Author(s):                              D Lima
#--------------------------------------------------------

SET(SRC
  SignalK.cpp
  SignalK_Info.cpp
  main.cpp
)

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

#Find MOOSGeodesy
find_package(MOOSGeodesy)
include_directories(${MOOSGeodesy_INCLUDE_DIRS} ${MOOSGeodesy_LIBRARIES})

