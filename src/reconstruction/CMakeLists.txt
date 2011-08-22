include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_ros_package(pcl) #find the ros version of pcl for now.

if(NOT pcl_FOUND)
message(STATUS "pcl could not be found, so not building surfels.")
return()
endif()

#deal with the objcog module
ectomodule(reconstruction
  module.cpp
  surfel_conversion.cpp
  surfels.cpp
  SurfelReconstruction.cpp
  reconstruction_db.cpp
  )

set_target_properties(reconstruction_ectomodule
      PROPERTIES
      COMPILE_FLAGS "-Wno-unknown-pragmas"
      )

link_ecto(reconstruction
  ${pcl_LIBRARIES}
  ${OpenCV_LIBS}
  object_recognition_db
  )

install_ecto_module(reconstruction)
