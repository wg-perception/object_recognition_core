# deal with the detection TOD module

add_library(tod_common
  SHARED
  maximum_clique.cpp
  adjacency_ransac.cpp
  )

target_link_libraries(tod_common
  ${OpenCV_LIBS}
  ${pcl_LIBRARIES}
#  ${Boost_LIBRARIES}
#  opencv_candidate
)
