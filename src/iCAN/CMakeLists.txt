#--------------------------------------------------------
# The CMakeLists.txt for:                       iCAN
# Author(s):                              DL
#--------------------------------------------------------

SET(SRC
  CAN.cpp
  CAN_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iCAN ${SRC})

TARGET_LINK_LIBRARIES(iCAN
   canprofile
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

