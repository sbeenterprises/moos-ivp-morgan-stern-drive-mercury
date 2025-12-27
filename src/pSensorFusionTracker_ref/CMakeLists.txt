#--------------------------------------------------------
# The CMakeLists.txt for:                       pSensorFusionTracker_ref
# Author(s):                              DL
#--------------------------------------------------------

cmake_minimum_required(VERSION 3.5)
project(pSensorFusionTracker_ref)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find MOOS
find_package(MOOS 10.0 REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${MOOS_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    .
)

SET(SRC
  SensorFusionTracker_ref.cpp
  SensorFusionTracker_ref_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pSensorFusionTracker_ref ${SRC})

TARGET_LINK_LIBRARIES(pSensorFusionTracker_ref
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

