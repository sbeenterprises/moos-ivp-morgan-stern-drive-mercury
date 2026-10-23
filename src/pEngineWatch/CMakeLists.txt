#--------------------------------------------------------
# The CMakeLists.txt for:                   pEngineWatch
#--------------------------------------------------------

SET(SRC
  EngineWatchModel.cpp
  EngineWatch.cpp
  EngineWatch_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pEngineWatch ${SRC})

find_package(CURL REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)

include_directories(${CURL_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})

# EngineWatchModel is plain C++ (unit tested in tests/enginewatch_test.cpp);
# only the MOOS app wrapper needs MOOS, curl and jsoncpp.
TARGET_LINK_LIBRARIES(pEngineWatch
   ${MOOS_LIBRARIES}
   ${CURL_LIBRARIES}
   ${JSONCPP_LIBRARIES}
   jsoncpp_lib
   apputil
   mbutil
   m
   pthread)
