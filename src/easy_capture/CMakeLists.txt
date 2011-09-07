find_package(ecto_pcl REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
  ${ecto_pcl_INCLUDE_DIRS}
  ${pcl_INCLUDE_DIRS}
  )

#deal with the objcog module
ectomodule(easy_capture
  module.cpp
  SamplePclFilter.cpp
  )

link_ecto(easy_capture
  ${pcl_LIBRARIES}
  )

install_ecto_module(easy_capture)
