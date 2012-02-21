# deal with the detection TOD module

find_package(ROS REQUIRED geometry_msgs rosbag roscpp sensor_msgs visualization_msgs)
find_package(ecto_ros REQUIRED)

include_directories(${ecto_ros_INCLUDE_DIRS})

ectomodule(io_ros
    module.cpp
    PoseArrayAssembler.cpp
    visualization_msgs_MarkerArray.cpp
    )

link_ecto(io_ros
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${OpenCV_LIBS}
    ${rosbag_LIBRARIES}
    ${roscpp_LIBRARIES}
    object_recognition_core_db
    object_recognition_core_common
)

install_ecto_module(io_ros)
