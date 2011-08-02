find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

add_library(opencv_candidate SHARED
  Camera.cpp
  PoseRT.cpp
  lsh.cpp
  )

target_link_libraries(opencv_candidate
  ${OpenCV_LIBS}
  )
