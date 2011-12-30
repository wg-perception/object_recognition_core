# deal with the detection TOD module
ectomodule(voters
  Aggregator.cpp
  module.cpp
  )

link_ecto(voters
  ${OpenCV_LIBS}
  object_recognition_db
  ${pcl_LIBRARIES}
  ${Boost_LIBRARIES}
  opencv_candidate
  )

install_ecto_module(voters)
