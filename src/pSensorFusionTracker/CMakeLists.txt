#==============================================================
# CMakeLists.txt for pSensorFusionTracker (with dlib)
#==============================================================

cmake_minimum_required(VERSION 3.5)
project(pSensorFusionTracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find MOOS
find_package(MOOS 10.0 REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find dlib (optional - comment out if not using)
find_package(dlib QUIET)
if(dlib_FOUND)
    add_definitions(-DUSE_DLIB)
    message(STATUS "dlib found - using optimal Hungarian algorithm")
else()
    message(STATUS "dlib not found - using built-in Hungarian algorithm")
endif()

# Include Directories
include_directories(
    ${MOOS_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    .
)

# Source Files
set(SRC
    pSensorFusionTracker.cpp
)

# Create Executable
add_executable(pSensorFusionTracker ${SRC})

# Link Libraries
target_link_libraries(pSensorFusionTracker
    ${MOOS_LIBRARIES}
    pthread
)

if(dlib_FOUND)
    target_link_libraries(pSensorFusionTracker dlib::dlib)
endif()

# Install
install(TARGETS pSensorFusionTracker
    RUNTIME DESTINATION bin
)