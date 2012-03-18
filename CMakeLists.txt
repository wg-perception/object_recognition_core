cmake_minimum_required(VERSION 2.8)
project(object_recognition_core)
find_package(catkin)
catkin_stack()
catkin_project(object_recognition_core
               DEPENDS Eigen Boost Opencv
               INCLUDE include
               LIBRARIES object_recognition_core_db object_recognition_core_common
)

include(cmake/test.cmake)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif()

add_definitions("-Wno-pragmas -fno-strict-aliasing -Wall -Wl,--no-undefined -Werror")

#set the default path for built libraries to the lib directory
if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

set(python_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${PYTHON_PACKAGES_PATH}/object_recognition_core)

find_package(Boost REQUIRED system filesystem serialization)
find_package(ecto REQUIRED)
find_package(OpenCV REQUIRED)

set_ecto_install_package_name(object_recognition_core/ecto_cells)

#install targets for all things python
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/
  DESTINATION ${PYTHON_PACKAGES_PATH}
  COMPONENT main
)

add_subdirectory(python)

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

# generate the messages and the actionlib server for ROS
find_package(genmsg REQUIRED)

if (genmsg_FOUND)
    if (ROS_ELECTRIC_FOUND)
        find_ros_package(actionlib_msgs)
        find_ros_package(geometry_msgs)
        find_ros_package(sensor_msgs)
        find_ros_package(visualization_msgs)
        include_directories(SYSTEM
                            ${ecto_ros_INCLUDE_DIRS}
        )
    else()
        find_package(ROS REQUIRED actionlib_msgs geometry_msgs rosbag roscpp sensor_msgs visualization_msgs)
    endif()

    add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/io/ros/msg FILES RecognizedObject.msg RecognizedObjectArray.msg Shape.msg)
#    add_action_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/server/action FILES ObjectRecognition.action)

    generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs sensor_msgs std_msgs visualization_msgs)
    # generate ecto cells to wrap the different messages
    #pubsub_gen_wrap(object_recognition_core)
endif()

catkin_add_env_hooks(10.object_recognition_core SHELLS all )
