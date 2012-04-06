cmake_minimum_required(VERSION 2.8)
project(object_recognition_core)
find_package(catkin)
catkin_stack()
catkin_project(object_recognition_core
               DEPENDS Eigen Boost OpenCV
               INCLUDE_DIRS include
               LIBRARIES object_recognition_core_db object_recognition_core_common
               CFG_EXTRAS object_recognition_core-extras.cmake
)

#install targets for all things python
catkin_python_setup()

find_package(ecto REQUIRED)
set_ecto_install_package_name(object_recognition_core/ecto_cells)

add_subdirectory(python)

include(cmake/test.cmake)
include(cmake/object_recognition_core-extras.cmake.in)

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

install(DIRECTORY apps/
        DESTINATION bin
        USE_SOURCE_PERMISSIONS
        PATTERN "*.bag" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
        )

#these setup the lib to be used by others
include(cmake/install.cmake)

# add tests
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

# generate ecto cells to wrap the different messages
find_package(ecto_ros REQUIRED)
pubsub_gen_wrap(object_recognition_msgs)

catkin_add_env_hooks(10.object_recognition_core SHELLS all)

