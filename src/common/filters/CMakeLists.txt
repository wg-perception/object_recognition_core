# deal with the detection TOD module
ectomodule(filters
  depth_filter.cpp
  module.cpp
  )

link_ecto(filters
  ${OpenCV_LIBS}
  object_recognition_db
  ${pcl_LIBRARIES}
  ${Boost_LIBRARIES}
  opencv_candidate
  )

install_ecto_module(filters)
