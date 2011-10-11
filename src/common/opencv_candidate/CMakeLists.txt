add_library(opencv_candidate SHARED
  lsh.cpp
  )

target_link_libraries(opencv_candidate
  ${OpenCV_LIBS}
  gomp
  )

set_target_properties(opencv_candidate PROPERTIES COMPILE_FLAGS "-msse4.2 -fopenmp")
