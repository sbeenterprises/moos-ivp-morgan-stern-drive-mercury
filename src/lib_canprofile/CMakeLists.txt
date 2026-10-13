#--------------------------------------------------------
# The CMakeLists.txt for:                 lib_canprofile
#   Per-boat CAN drive profiles (no MOOS dependency)
#--------------------------------------------------------

ADD_LIBRARY(canprofile STATIC
  CanProfile.cpp
  DriveCore.cpp
)
