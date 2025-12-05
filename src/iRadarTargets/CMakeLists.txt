#--------------------------------------------------------
# The CMakeLists.txt for:                       iRadarTargets
# Author(s):                              DL
#--------------------------------------------------------

SET(SRC
  RadarTargets.cpp
  RadarTargets_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iRadarTargets ${SRC})

TARGET_LINK_LIBRARIES(iRadarTargets
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

