include_directories(${CMAKE_CURRENT_SOURCE_DIR})
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})
find_ros_package(pcl) #find the ros version of pcl for now.
add_definitions("-Wno-unknown-pragmas")

if(NOT pcl_FOUND)
message(STATUS "pcl could not be found, so not building surfels.")
return()
endif()

#deal with the objcog module
ectomodule(reconstruction
  module.cpp
  surfel_conversion.cpp
  surfels.cpp
  )

link_ecto(reconstruction
  object_recognition_db
  ${OpenCV_LIBS}
  )

install_ecto_module(reconstruction)
