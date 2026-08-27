#=======================================================================
# FILE:  moos-ivp-extend/CMakeLists.txt
# DATE:  2012/07/24
# INFO:  Top-level CMakeLists.txt file for the moos-ivp-extend project
# NAME:  Maintained by Mike Benjamin - Original setup by Christian Convey
#        Chris Gagner, and tips borrowed from Dave Billin
#=======================================================================

CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT( IVP_EXTEND )

set (CMAKE_CXX_STANDARD 11)

#=======================================================================
# Set the output directories for the binary and library files
#=======================================================================

GET_FILENAME_COMPONENT(IVP_EXTEND_BIN_DIR "${CMAKE_SOURCE_DIR}/bin"  ABSOLUTE )
GET_FILENAME_COMPONENT(IVP_EXTEND_LIB_DIR "${CMAKE_SOURCE_DIR}/lib"  ABSOLUTE )

SET( LIBRARY_OUTPUT_PATH      "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )
SET( ARCHIVE_OUTPUT_DIRECTORY "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )
SET( LIBRARY_OUTPUT_DIRECTORY "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )

SET( EXECUTABLE_OUTPUT_PATH    "${IVP_EXTEND_BIN_DIR}" CACHE PATH "" )
SET( RUNTIME_OUTPUT_DIRECTORY "${IVP_EXTEND_BIN_DIR}"  CACHE PATH "" )

#=======================================================================
# Find MOOS
#=======================================================================
# Set MOOS installation path for the other computer
if(EXISTS "/home/usv/moos-ivp")
    set(CMAKE_PREFIX_PATH "/home/usv/moos-ivp" ${CMAKE_PREFIX_PATH})
endif()

find_package(MOOS 10.0)
INCLUDE_DIRECTORIES(${MOOS_INCLUDE_DIRS})

#=======================================================================
# Find the "moos-ivp" base directory
#=======================================================================

# Search for the moos-ivp folder
find_path( MOOSIVP_SOURCE_TREE_BASE
           NAMES build-ivp.sh build-moos.sh configure-ivp.sh
           PATHS "../moos-ivp" "../../moos-ivp" "../../moos-ivp/trunk/" "../moos-ivp/trunk/" "/home/d2snc/moos-ivp" "/home/usv/moos-ivp"
           DOC "Base directory of the MOOS-IvP source tree"
           NO_DEFAULT_PATH
)

if (NOT MOOSIVP_SOURCE_TREE_BASE)
    message("Please set MOOSIVP_SOURCE_TREE_BASE to  ")
    message("the location of the \"moos-ivp\" folder ")
    message( FATAL_ERROR "CMake will exit." )
endif()

#======================================================================
# Specify where to find IvP's headers and libraries...
#======================================================================

FILE(GLOB IVP_INCLUDE_DIRS ${MOOSIVP_SOURCE_TREE_BASE}/ivp/src/lib_* )
INCLUDE_DIRECTORIES(${IVP_INCLUDE_DIRS})

FILE(GLOB IVP_LIBRARY_DIRS ${MOOSIVP_SOURCE_TREE_BASE}/lib )
LINK_DIRECTORIES(${IVP_LIBRARY_DIRS})

#======================================================================
# Specify Compiler Flags
#======================================================================
IF( ${WIN32} )
   #---------------------------------------------
   # Windows Compiler Flags
   #---------------------------------------------
   IF(MSVC)
      # Flags for Microsoft Visual Studio
      SET( WALL_ON OFF CACHE BOOL 
         "tell me about all compiler warnings (-Wall) ")
      IF(WALL_ON)
         SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
      ENDIF(WALL_ON)
   ELSE(MSVC)
      # Other Windows compilers go here
   ENDIF(MSVC)

ELSE( ${WIN32} )
   #---------------------------------------------
   # Linux and Apple Compiler Flags
   #---------------------------------------------
   # Force -fPIC because gcc complains when we don't use it with x86_64 code.
   # Note sure why: -fPIC should only be needed for shared objects, and
   # AFAIK, CMake gets that right when building shared objects. -CJC
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -g -Wdeprecated-declarations")
   IF(CMAKE_COMPILER_IS_GNUCXX)
      # Flags for the GNU C++ Compiler
      SET( WALL_ON OFF CACHE BOOL 
         "tell me about all compiler warnings (-Wall) ")
      IF(WALL_ON)
         SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall" -C++11)
      ENDIF( WALL_ON)
   ELSE(CMAKE_COMPILER_IS_GNUCXX)
    
   ENDIF(CMAKE_COMPILER_IS_GNUCXX)

ENDIF( ${WIN32} )

#======================================================================
# Local grid projection - must match the moos-ivp core build
#======================================================================
# moos-ivp/ivp/src/CMakeLists.txt declares this same option, default ON, and
# pMarineViewer, pNodeReporter and the chart alignment in lib_marineview are
# all compiled against it. Every app here that converts between lat/lon and
# MOOS x/y has to choose the same projection or it ends up in a different
# frame: at Morgan City, ~6 km from the origin, UTM and the flat-earth local
# grid disagree by about 100 m. Flip this only together with the core's flag.
SET(USE_UTM ON CACHE BOOL "Use UTM coordinates vs Local coordinates.")

IF("${USE_UTM}" STREQUAL "ON")
  ADD_DEFINITIONS(-DUSE_UTM)
ENDIF()

message("----------------------------------------Use UTM:" ${USE_UTM})

#=======================================================================
# Add Subdirectories
#=======================================================================
# tools/ comes first on purpose. src/pVisionDetector_ref does not build on
# this machine (it needs openvino/openvino.hpp), and build.sh runs a plain
# `make`, which stops at the first failure - so anything listed after src/
# would silently never be built.
ADD_SUBDIRECTORY( tools )

ADD_SUBDIRECTORY( src )
