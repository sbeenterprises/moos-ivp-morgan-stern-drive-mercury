#--------------------------------------------------------
# The CMakeLists.txt for:                       iRaspberryServer
# Author(s):                              Douglas Lima
#--------------------------------------------------------

SET(SRC
  RaspberryServer.cpp
  RaspberryServer_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iRaspberryServer ${SRC})

TARGET_LINK_LIBRARIES(iRaspberryServer
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

