#--------------------------------------------------------
# The CMakeLists.txt for:                     pTurnLimit
#--------------------------------------------------------

SET(SRC
  TurnLimitModel.cpp
  TurnLimit.cpp
  TurnLimit_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pTurnLimit ${SRC})

# TurnLimitModel is plain C++ (unit tested in tests/turnlimit_test.cpp);
# only the MOOS app wrapper needs the MOOS libraries.
TARGET_LINK_LIBRARIES(pTurnLimit
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)
