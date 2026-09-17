#--------------------------------------------------------
# The CMakeLists.txt for:                    pColavEval
# Author(s):                                Douglas Lima
#--------------------------------------------------------

SET(SRC
  ColavEval.cpp
  ColavEval_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pColavEval ${SRC})

# geometry brings CPAEngine, used for the predicted CPA / TCPA. The achieved
# closest range is measured directly and needs nothing.
TARGET_LINK_LIBRARIES(pColavEval
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread)
