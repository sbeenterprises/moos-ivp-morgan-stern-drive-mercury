#--------------------------------------------------------
# The CMakeLists.txt for:                       iCANRec
# Author(s):                              DL
#--------------------------------------------------------

SET(SRC
  CANRec.cpp
  CANRec_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iCANRec ${SRC})

TARGET_LINK_LIBRARIES(iCANRec
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

