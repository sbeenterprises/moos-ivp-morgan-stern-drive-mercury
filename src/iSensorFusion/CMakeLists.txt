#--------------------------------------------------------
# The CMakeLists.txt for:                       iSensorFusion
# Author(s):                              DL
#--------------------------------------------------------

SET(SRC
  SensorFusion.cpp
  SensorFusion_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iSensorFusion ${SRC})

TARGET_LINK_LIBRARIES(iSensorFusion
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

