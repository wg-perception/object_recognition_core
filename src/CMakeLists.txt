find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

add_subdirectory(capture)
add_subdirectory(common)
add_subdirectory(db)
add_subdirectory(tod)
add_subdirectory(reconstruction)
