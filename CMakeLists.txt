cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "The build type.")
project(object_recognition)
add_definitions("-Wno-pragmas -fno-strict-aliasing -Wall -Wl,--no-undefined")

#set the default path for built libraries to the "ecto_opencv" directory
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/ecto_object_recognition)

find_package(Boost COMPONENTS system filesystem serialization REQUIRED)
find_package(ecto 0.1.0 REQUIRED)
find_package(OpenCV 2.3.1 EXACT REQUIRED)

set_ecto_install_package_name(ecto_object_recognition)

configure_file(python/__init__.py.in
  ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
  @ONLY)

#path setup for executables
set(APP_PATH ${CMAKE_CURRENT_SOURCE_DIR}/apps)
configure_file( apps/setup.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/setup.sh
  @ONLY
  )

ecto_python_env_gen(${CMAKE_BINARY_DIR}/lib ${CMAKE_CURRENT_SOURCE_DIR}/python)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_subdirectory(src)
