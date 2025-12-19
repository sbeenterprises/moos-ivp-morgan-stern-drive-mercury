# Install script for directory: //home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "None")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/lib_behaviors-test/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/pXRelayTest/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/iDivisorNMEA/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/planchaPID/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/iNMEA/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/iSignalK/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/iWebSocket/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/pXYpublisher/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/iCAN/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/iCANRec/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/pContactSpawn/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/iRadarTargets/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/pSensorFusionTracker_ref/cmake_install.cmake")
  include("//home/douglas/Documents/automation/moos-ivp-morgan-stern-drive-mercury/build/src/pVisionDetector_ref/cmake_install.cmake")

endif()

