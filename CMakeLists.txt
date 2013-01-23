cmake_minimum_required(VERSION 2.8)
project(object_recognition_core)

find_package(catkin)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
find_package(catkin REQUIRED ecto)
catkin_package(DEPENDS ecto
               INCLUDE_DIRS include
               LIBRARIES object_recognition_core_db object_recognition_core_common
               CFG_EXTRAS test.cmake
)
else()
find_package(ROS REQUIRED ecto)
catkin_stack()
catkin_project(object_recognition_core
               INCLUDE_DIRS include
               LIBRARIES object_recognition_core_db object_recognition_core_common
               CFG_EXTRAS test.cmake
)
set(catkin_LIBRARIES ${ROS_LIBRARIES})
set(catkin_INCLUDE_DIRS ${ROS_INCLUDE_DIRS})
set(CATKIN_DEVEL_PREFIX ${CMAKE_BINARY_DIR})
set(CATKIN_PACKAGE_BIN_DESTINATION share/${PROJECT_NAME}/bin)
set(CATKIN_PACKAGE_INCLUDE_DESTINATION include/${PROJECT_NAME})
set(CATKIN_PACKAGE_LIB_DESTINATION lib)
set(CATKIN_PACKAGE_PYTHON_BUILD_DESTINATION gen/py/${PROJECT_NAME}/)
set(CATKIN_PACKAGE_PYTHON_DESTINATION ${INSTALLED_PYTHONPATH}/${PROJECT_NAME}/)
set(CATKIN_PACKAGE_SHARE_DESTINATION share/${PROJECT_NAME}/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

#install targets for all things python
catkin_python_setup()

add_subdirectory(python)

add_definitions("-Wno-pragmas -fno-strict-aliasing -Wall -Wl,--no-undefined -Werror")

find_package(Boost REQUIRED system filesystem serialization)
find_package(OpenCV REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)
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
