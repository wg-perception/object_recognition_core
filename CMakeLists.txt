cmake_minimum_required(VERSION 2.8)
project(object_recognition_core)
find_package(catkin REQUIRED ecto)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
catkin_package(DEPENDS ecto
               INCLUDE_DIRS include
               LIBRARIES object_recognition_core_db object_recognition_core_common
               CFG_EXTRAS test.cmake
)
else()
catkin_stack()
catkin_project(object_recognition_core
               INCLUDE_DIRS include
               LIBRARIES object_recognition_core_db object_recognition_core_common
               CFG_EXTRAS test.cmake
)
endif()

#install targets for all things python
catkin_python_setup()

add_subdirectory(python)

add_definitions("-Wno-pragmas -fno-strict-aliasing -Wall -Wl,--no-undefined -Werror")

#set the default path for built libraries to the lib directory
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

find_package(Boost REQUIRED system filesystem serialization)
find_package(OpenCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(src)
add_subdirectory(web_ui)

#these setup the lib to be used by others
include(cmake/install.cmake)

# add tests
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

catkin_doxygen(object_recognition_core-doxygen ${CMAKE_CURRENT_SOURCE_DIR}/include)
