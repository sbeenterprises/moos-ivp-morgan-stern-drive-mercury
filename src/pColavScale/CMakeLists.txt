#--------------------------------------------------------
# The CMakeLists.txt for:                    pColavScale
#--------------------------------------------------------

SET(SRC
  ColavScaleModel.cpp
  ColavScale.cpp
  ColavScale_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pColavScale ${SRC})

# ColavScaleModel is plain C++ (unit tested in tests/colavscale_test.cpp);
# only the MOOS app wrapper needs the MOOS libraries.
TARGET_LINK_LIBRARIES(pColavScale
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)
