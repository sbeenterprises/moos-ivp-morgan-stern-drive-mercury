#--------------------------------------------------------
# The CMakeLists.txt for:                       planchaPID
# Author(s):                              Douglas Lima
#--------------------------------------------------------

SET(SRC
  lanchaPID.cpp
  lanchaPID_Info.cpp
  main.cpp
  PIDParameterReader.cpp
)

ADD_EXECUTABLE(planchaPID ${SRC})

TARGET_LINK_LIBRARIES(planchaPID
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

