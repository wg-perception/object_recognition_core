add_library(opencv_candidate SHARED
  lsh.cpp
  )

target_link_libraries(opencv_candidate
  ${OpenCV_LIBS}
  )
