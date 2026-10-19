##############################################################################
# FILE:        moos-ivp-extend/src/CMakeLists.txt
# DATE:        2010/09/07
#              2020/05/09 minor mods
# DESCRIPTION: CMakeLists.txt file for the moos-ivp-extend source directory
##############################################################################


#============================================================================
# Add the libraries in the current directory to the include path
#============================================================================
FILE(GLOB LOCAL_LIBRARY_DIRS ./lib_*)
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})


#============================================================================
# List the subdirectories to build...
#============================================================================
ADD_SUBDIRECTORY(lib_behaviors-test)
ADD_SUBDIRECTORY(lib_canprofile)
ADD_SUBDIRECTORY(pXRelayTest)
ADD_SUBDIRECTORY(iDivisorNMEA)
ADD_SUBDIRECTORY(planchaPID)
ADD_SUBDIRECTORY(iNMEA)
ADD_SUBDIRECTORY(iSignalK)
ADD_SUBDIRECTORY(iWebSocket)
ADD_SUBDIRECTORY(pXYpublisher)
ADD_SUBDIRECTORY(iCAN)
ADD_SUBDIRECTORY(iCANRec)
ADD_SUBDIRECTORY(pContactSpawn)
ADD_SUBDIRECTORY(pTrafficSim)
ADD_SUBDIRECTORY(pColavEval)
ADD_SUBDIRECTORY(pCorridor)
ADD_SUBDIRECTORY(iRadarTargets)
ADD_SUBDIRECTORY(pSensorFusionTracker_ref)
#ADD_SUBDIRECTORY(pVisionDetector_ref)



##############################################################################
#                           END of CMakeLists.txt
##############################################################################

