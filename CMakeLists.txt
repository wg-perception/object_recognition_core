cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "The build type.")
project(object_recognition)
add_definitions("-Werror -fno-strict-aliasing -Wall -Wl,--no-undefined")
include(rosbuild_lite.cmake)

#set the default path for built libraries to the "ecto_opencv" directory
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

#use the cmake cache, to -DOpenCV_dir=<path_to_build_of_opencv>
#  svn co https://code.ros.org/svn/opencv/trunk
#  and cmake that baby
find_package(Boost COMPONENTS system filesystem serialization REQUIRED)
find_package(ecto 0.1.0 REQUIRED)
find_package(OpenCV 2.3.1 EXACT REQUIRED)

ecto_python_env_gen(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${EIGEN_INCLUDE_DIRS})
add_subdirectory(src)
