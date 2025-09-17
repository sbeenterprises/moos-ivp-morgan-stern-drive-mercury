#--------------------------------------------------------
# The CMakeLists.txt for:                       iCANReceiver
# Author(s):                              D Lima
#--------------------------------------------------------

SET(SRC
  CANReceiver.cpp
  CANReceiver_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iCANReceiver ${SRC})

TARGET_LINK_LIBRARIES(iCANReceiver
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

