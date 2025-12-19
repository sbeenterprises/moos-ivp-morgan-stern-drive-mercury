#--------------------------------------------------------
# The CMakeLists.txt for:                       pVisionDetector_ref
# Author(s):                              DL
#--------------------------------------------------------

SET(SRC
  VisionDetector_ref.cpp
  VisionDetector_ref_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pVisionDetector_ref ${SRC})

TARGET_LINK_LIBRARIES(pVisionDetector_ref
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread)

