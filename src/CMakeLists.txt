find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
find_ros_package(pcl) #find the ros version of pcl for now.
if(NOT pcl_FOUND)
message(FATAL_ERROR "pcl could not be found, so not building tod.")
endif()

add_subdirectory(capture)
add_subdirectory(common)
add_subdirectory(db)
add_subdirectory(tod)
add_subdirectory(reconstruction)
add_subdirectory(easy_capture)
