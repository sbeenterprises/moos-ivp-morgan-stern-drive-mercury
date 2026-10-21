#--------------------------------------------------------
# The CMakeLists.txt for:                      pNavGuard
#--------------------------------------------------------

SET(SRC
  NavGuardModel.cpp
  NavGuard.cpp
  NavGuard_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pNavGuard ${SRC})

# NavGuardModel is plain C++ (unit tested in tests/navguard_test.cpp);
# only the MOOS app wrapper needs the MOOS libraries.
TARGET_LINK_LIBRARIES(pNavGuard
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

# Offline check of a recorded .alog with the same model (no MOOS).
ADD_EXECUTABLE(navguard_replay NavGuardModel.cpp navguard_replay.cpp)
TARGET_LINK_LIBRARIES(navguard_replay m)
