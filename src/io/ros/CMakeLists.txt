# create the cells to output messages

find_package(ecto_ros QUIET)
if (NOT ecto_ros_FOUND)
  return()
endif()
find_package(object_recognition_msgs REQUIRED)

if (ROS_GROOVY_FOUND)
find_package(catkin REQUIRED geometry_msgs rosbag roscpp sensor_msgs visualization_msgs)
else()
find_package(ROS REQUIRED geometry_msgs rosbag roscpp sensor_msgs visualization_msgs)
endif()

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)

include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
                    ${ecto_ros_INCLUDE_DIRS}
                    ${geometry_msgs_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

ectomodule(io_ros DESTINATION object_recognition_core/ecto_cells
                  INSTALL
           module.cpp
           MsgAssembler.cpp
           visualization_msgs_MarkerArray.cpp
           VisualizationMsgAssembler.cpp
)

link_ecto(io_ros
          ${geometry_msgs_LIBRARIES}
          ${sensor_msgs_LIBRARIES}
          ${OpenCV_LIBRARIES}
          ${PCL_LIBRARIES}
          ${rosbag_LIBRARIES}
          ${roscpp_LIBRARIES}
          object_recognition_core_db
          object_recognition_core_common
)
