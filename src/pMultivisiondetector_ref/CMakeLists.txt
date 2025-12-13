cmake_minimum_required(VERSION 3.10)
project(pMultivisiondetector_ref)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find MOOS
find_package(MOOS 10.0 REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio dnn)

# Find OpenVINO
find_package(OpenVINO REQUIRED)

# Include directories
include_directories(
    ${MOOS_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add debug info
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")

# Source files
set(SRC
    main.cpp
    Multivisiondetector_ref.cpp
    Multivisiondetector_ref_Info.cpp
    YOLOInference.cpp
)

set(HEADERS
    Multivisiondetector_ref.h
    Multivisiondetector_ref_Info.h
    YOLOInference.h
)

# Create executable
add_executable(pMultivisiondetector_ref ${SRC} ${HEADERS})

# Link libraries
target_link_libraries(pMultivisiondetector_ref
    ${MOOS_LIBRARIES}
    ${OpenCV_LIBS}
    openvino::runtime
    apputil
    mbutil
    pthread
)

# Install
install(TARGETS pMultivisiondetector_ref
    RUNTIME DESTINATION bin
)
