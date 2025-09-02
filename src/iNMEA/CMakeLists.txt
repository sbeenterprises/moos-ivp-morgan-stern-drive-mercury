#--------------------------------------------------------
# The CMakeLists.txt for:                       iNMEA
# Author(s):                              DLima
#--------------------------------------------------------

SET(SRC
  NMEA.cpp
  NMEA_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iNMEA ${SRC})

TARGET_LINK_LIBRARIES(iNMEA
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

