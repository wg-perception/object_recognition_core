find_ros_package(pcl) #find the ros version of pcl for now.

if(NOT pcl_FOUND)
message(STATUS "pcl could not be found, so not building tod.")
return()
endif()

include_directories(
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${pcl_INCLUDE_DIRS}
                    )

# deal with the common TOD module
ectomodule(tod
  tod.cpp 
  TwoDToThreeD.cpp
  CameraToWorld.cpp
  )

link_ecto(tod
  ${OpenCV_LIBS}
  object_recognition_db
  ${pcl_LIBRARIES}
  ${Boost_LIBRARIES}
  opencv_candidate
  )

install_ecto_module(tod)

# deal with the detection TOD module
ectomodule(tod_detection
  detecting/csv.cpp
  detecting/descriptor_matcher.cpp
  detecting/guess_csv_writer.cpp
  detecting/guess_generator.cpp
  detecting/TodDetection.cpp
  )

link_ecto(tod_detection
  ${OpenCV_LIBS}
  object_recognition_db
  ${pcl_LIBRARIES}
  ${Boost_LIBRARIES}
  opencv_candidate
  )

install_ecto_module(tod_detection)

# deal with the training TOD modules
ectomodule(tod_training
    training/ModelInserter.cpp
    training/ModelStacker.cpp
    )

link_ecto(tod_training
  object_recognition_db
  ${OpenCV_LIBS}
  )

install_ecto_module(tod_training)
