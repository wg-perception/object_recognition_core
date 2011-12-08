add_library(opencv_candidate SHARED
  lsh.cpp
  )

target_link_libraries(opencv_candidate
  ${OpenCV_LIBS}
  )

#set_target_properties(opencv_candidate PROPERTIES COMPILE_FLAGS "-msse4.2")

install(TARGETS opencv_candidate
    DESTINATION lib
    )
set_target_properties(opencv_candidate PROPERTIES
  VERSION ${object_recognition_VERSION}
  SOVERSION ${object_recognition_SOVERSION}
  )