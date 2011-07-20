cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "The build type.")
project(objcog)
add_definitions("-Werror -fno-strict-aliasing -Wall -Wl,--no-undefined")

#set the default path for built libraries to the "ecto_opencv" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

#use the cmake cache, to -DOpenCV_dir=<path_to_build_of_opencv>
#  svn co https://code.ros.org/svn/opencv/trunk
#  and cmake that baby
find_package(Boost COMPONENTS system filesystem serialization REQUIRED)
find_package(ecto 0.1.0 REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV 2.3.1 EXACT REQUIRED)
find_package(PCL REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
ecto_python_env_gen(${CMAKE_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_subdirectory(src)
