#--------------------------------------------------------
# The CMakeLists.txt for: pVisionDetector
# Author(s): Abdul Aziz
#--------------------------------------------------------

cmake_minimum_required(VERSION 3.5)
project(pVisionDetector)

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
    .
)

# Source files
set(SRC
    VisionDetector.cpp
    YOLOInference.cpp
)

# Create executable
add_executable(pVisionDetector ${SRC})

# Link libraries (removed IVP libraries - we don't need them)
target_link_libraries(pVisionDetector
    ${MOOS_LIBRARIES}
    ${OpenCV_LIBS}
    ${OpenVINO_LIBRARIES}
    pthread
)

# Install
install(TARGETS pVisionDetector
    RUNTIME DESTINATION bin
)
