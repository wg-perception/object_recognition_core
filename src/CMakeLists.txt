
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
find_package(PCL)

if(NOT PCL_FOUND)
message(FATAL_ERROR "pcl could not be found, so not building tod.")
endif()

add_subdirectory(common)
add_subdirectory(db)
add_subdirectory(reconstruction)
