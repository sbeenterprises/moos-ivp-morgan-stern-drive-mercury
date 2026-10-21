#--------------------------------------------------------
# The CMakeLists.txt for:                 lib_regiongeom
#--------------------------------------------------------
# Operating-region geometry (plain C++, unit tested in
# tests/region_test.cpp); used by iWebSocket.

SET(SRC
  RegionGeom.cpp
)

ADD_LIBRARY(regiongeom STATIC ${SRC})
