cmake_minimum_required(VERSION 2.8)
project(object_recognition_core)
include(cmake/config.cmake)
include(cmake/test.cmake)
include(cmake/version.cmake)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif()

add_definitions("-Wno-pragmas -fno-strict-aliasing -Wall -Wl,--no-undefined")

#set the default path for built libraries to the "ecto_opencv" directory
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

set(python_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PYTHON_PACKAGES_PATH}/object_recognition_core)

find_package(Boost REQUIRED system filesystem serialization)
find_package(ecto REQUIRED)
find_package(OpenCV REQUIRED)

set_ecto_install_package_name(object_recognition_core)

#install targets for all things python
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/
  DESTINATION ${PYTHON_PACKAGES_PATH}
  COMPONENT main
)

configure_file(python/__init__.py.in
               ${python_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
               @ONLY
)

install(FILES ${python_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_core
)

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

catkin_add_env_hooks(10object_recognition_core SHELLS all )

# add tests
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

# generate the messages and the actionlib server for ROS
find_package(genmsg REQUIRED)

if (genmsg_FOUND)
    if (ROS_ELECTRIC_FOUND)
        find_ros_package(actionlib_msgs)
        find_ros_package(geometry_msgs)
        find_ros_package(sensor_msgs)
        find_ros_package(visualization_msgs)
        include_directories(${ecto_ros_INCLUDE_DIRS})
    else()
        find_package(ROS REQUIRED actionlib_msgs geometry_msgs rosbag roscpp sensor_msgs visualization_msgs)
    endif()

    add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/io/ros/msg FILES ObjectRecognition.msg ObejctRecognitionScene.msg ObjectRecognitionSceneArray.msg)
#    add_action_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/server/action FILES ObjectRecognition.action)

    generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs sensor_msgs std_msgs visualization_msgs)
    # generate ecto cells to wrap the different messages
    #pubsub_gen_wrap(object_recognition_core)
endif()
