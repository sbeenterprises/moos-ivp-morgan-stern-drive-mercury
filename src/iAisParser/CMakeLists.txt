#--------------------------------------------------------
# The CMakeLists.txt for:                       iAisParser
# Author(s):                              DL
#--------------------------------------------------------

SET(SRC
  AisParser.cpp
  AisParser_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(iAisParser ${SRC})

TARGET_LINK_LIBRARIES(iAisParser
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

#Find MOOSGeodesy
find_package(MOOSGeodesy)
include_directories(${MOOSGeodesy_INCLUDE_DIRS} ${MOOSGeodesy_LIBRARIES})

