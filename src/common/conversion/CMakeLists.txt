add_subdirectory(ros)

ectomodule(conversion
  PointCloudToMat.cpp
  module.cpp
  )

link_ecto(conversion
  ${OpenCV_LIBS}
  ${pcl_LIBRARIES}
  ${Boost_LIBRARIES}
  )

install_ecto_module(conversion)
