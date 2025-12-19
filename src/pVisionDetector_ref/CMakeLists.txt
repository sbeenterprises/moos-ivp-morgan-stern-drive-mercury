#--------------------------------------------------------
# The CMakeLists.txt for:                       pVisionDetector_ref
# Author(s):                              DL
#--------------------------------------------------------

cmake_minimum_required(VERSION 3.5)
project(pVisionDetector_ref)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find MOOS
find_package(MOOS 10.0 REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# OpenVINO
set(OpenVINO_INCLUDE_DIRS "/usr/include/openvino")
set(OpenVINO_LIBRARIES "/usr/lib/libopenvino.so")

# Include directories
include_directories(
    ${MOOS_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OpenVINO_INCLUDE_DIRS}
    ../pVisionDetector
    .
)

SET(SRC
  VisionDetector_ref.cpp
  VisionDetector_ref_Info.cpp
  main.cpp
  ../pVisionDetector/YOLOInference.cpp
)

ADD_EXECUTABLE(pVisionDetector_ref ${SRC})

TARGET_LINK_LIBRARIES(pVisionDetector_ref
   ${MOOS_LIBRARIES}
   ${OpenCV_LIBS}
   ${OpenVINO_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

