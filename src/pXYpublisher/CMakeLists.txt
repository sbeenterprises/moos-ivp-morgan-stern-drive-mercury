#--------------------------------------------------------
# The CMakeLists.txt for:                       pXYpublisher
# Author(s):                              D Lima
#--------------------------------------------------------

SET(SRC
  XYpublisher.cpp
  XYpublisher_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pXYpublisher ${SRC})

TARGET_LINK_LIBRARIES(pXYpublisher
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

