#--------------------------------------------------------
# The CMakeLists.txt for:                       pMultisensorfusion
# Author(s):                              Abdul Aziz
#--------------------------------------------------------

SET(SRC
  Multisensorfusion.cpp
  Multisensorfusion_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pMultisensorfusion ${SRC})

TARGET_LINK_LIBRARIES(pMultisensorfusion
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

