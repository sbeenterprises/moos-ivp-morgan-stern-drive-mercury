#--------------------------------------------------------
# The CMakeLists.txt for:                     pCorridor
#--------------------------------------------------------

SET(SRC
  CorridorModel.cpp
  Corridor.cpp
  Corridor_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pCorridor ${SRC})

# CorridorModel is plain C++ (unit tested in tests/corridor_test.cpp);
# only the MOOS app wrapper needs the MOOS libraries.
TARGET_LINK_LIBRARIES(pCorridor
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)
