# create the cells to output messages
rosbuild_lite_init()

find_package(ecto_ros QUIET)
if (NOT ecto_ros_FOUND)
  return()
endif()
find_package(object_recognition_msgs REQUIRED)

if (ROS_ELECTRIC_FOUND)
  find_ros_package(geometry_msgs)
  find_ros_package(sensor_msgs)
  find_ros_package(visualization_msgs) #find the ros version of pcl for now.
  include_directories(SYSTEM
                      ${ecto_ros_INCLUDE_DIRS}
  )
else()
if (GROOVY_FOUND)
find_package(catkin REQUIRED geometry_msgs rosbag roscpp sensor_msgs visualization_msgs)
else()
if (ROS_GROOVY_FOUND)
find_package(catkin REQUIRED geometry_msgs rosbag roscpp sensor_msgs visualization_msgs)
else()
find_package(ROS REQUIRED geometry_msgs rosbag roscpp sensor_msgs visualization_msgs)
endif()
endif()
endif()

find_package(Eigen REQUIRED)
find_pcl_package()

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
