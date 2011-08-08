find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

find_ros_package(pcl) #find the ros version of pcl for now.

if(NOT pcl_FOUND)
message(STATUS "pcl could not be found, so not building tod.")
return()
endif()

include_directories(
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${pcl_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIR}
                    )

# deal with the TOD module
ectomodule(tod
  tod.cpp 
  twoDToThreeD.cpp
  cameraToWorld.cpp
  detecting/csv.cpp
  detecting/guess_generator.cpp
  detecting/guess_csv_writer.cpp
  detecting/descriptor_matcher.cpp
  )

link_ecto(tod
  ${OpenCV_LIBS}
  object_recognition_db
  ${pcl_LIBRARIES}
  ${Boost_LIBRARIES}
  opencv_candidate
  )

install_ecto_module(tod)

#deal with the db part of TOD
ectomodule(tod_db
    tod_db.cpp
    )

link_ecto(tod_db
  object_recognition_db
  ${OpenCV_LIBS}
  )
install_ecto_module(tod_db)
