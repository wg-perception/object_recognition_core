# deal with the detection TOD module

find_ros_package(geometry_msgs)
find_ros_package(sensor_msgs)
find_ros_package(visualization_msgs) #find the ros version of pcl for now.
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
    )

install_ecto_module(io_ros)
