#--------------------------------------------------------
# The CMakeLists.txt for:                       pSensorFusionTracker_ref
# Author(s):                              DL
#--------------------------------------------------------

SET(SRC
  SensorFusionTracker_ref.cpp
  SensorFusionTracker_ref_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pSensorFusionTracker_ref ${SRC})

TARGET_LINK_LIBRARIES(pSensorFusionTracker_ref
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

