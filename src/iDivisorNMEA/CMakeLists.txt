#--------------------------------------------------------
# The CMakeLists.txt for:                       iDivisorNMEA
# Author(s):                              Douglas Lima
#--------------------------------------------------------

SET(SRC
  DivisorNMEA.cpp
  DivisorNMEA_Info.cpp
  AisDecode.cpp
  main.cpp
)
add_subdirectory (NMEAParserLib)
add_subdirectory (libais)
# Find MOOSGeodesy before linking: newer MOOS-IvP provides it as an
# imported target, so ${MOOSGeodesy_LIBRARIES} must be set before
# TARGET_LINK_LIBRARIES (it was empty when found afterwards).
find_package(MOOSGeodesy)
include_directories(${MOOSGeodesy_INCLUDE_DIRS})

ADD_EXECUTABLE(iDivisorNMEA ${SRC})

TARGET_LINK_LIBRARIES(iDivisorNMEA
   ${MOOS_LIBRARIES}
   ${MOOSGeodesy_LIBRARIES}
   apputil
   mbutil
   m
   pthread)
