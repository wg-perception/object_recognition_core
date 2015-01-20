cmake_minimum_required(VERSION 2.8)
project(object_recognition_core)

find_package(catkin REQUIRED cmake_modules ecto)
catkin_package(DEPENDS ecto
               INCLUDE_DIRS include
               LIBRARIES object_recognition_core_db object_recognition_core_common
               CFG_EXTRAS test.cmake
)

#install targets for all things python
catkin_python_setup()

add_subdirectory(python)

add_definitions("-Wno-pragmas -fno-strict-aliasing -Wall -Wl,--no-undefined -Werror")

find_package(Boost REQUIRED system filesystem serialization)
find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
 add_definitions("-DOPENCV3=1")
endif()

include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(src)
add_subdirectory(web_ui)

#these setup the lib to be used by others
include(cmake/install.cmake)

# add tests
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# build docs
add_subdirectory(doc)
catkin_doxygen(object_recognition_core-doxygen ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Use the ecto scripts to build top-level documentation
set(ecto_SPHINX_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/source)
ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source doc)
