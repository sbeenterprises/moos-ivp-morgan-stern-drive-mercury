#--------------------------------------------------------
# The CMakeLists.txt for:                       pTrafficSim
# Author(s):                              Douglas Lima
#--------------------------------------------------------

#Find MOOSGeodesy
find_package(MOOSGeodesy)
include_directories(${MOOSGeodesy_INCLUDE_DIRS})

SET(SRC
  TrafficSim.cpp
  TrafficSim_Info.cpp
  AisEncode.cpp
  main.cpp
)

ADD_EXECUTABLE(pTrafficSim ${SRC})

TARGET_LINK_LIBRARIES(pTrafficSim
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   apputil
   mbutil
   m
   pthread)
