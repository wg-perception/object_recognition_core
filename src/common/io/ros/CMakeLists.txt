# deal with the detection TOD module

find_ros_package(geometry_msgs)
find_ros_package(sensor_msgs)

ectomodule(io_ros
    module.cpp
    PoseArrayAssembler.cpp
    )

link_ecto(io_ros
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${OpenCV_LIBS}
    )

install_ecto_module(io_ros)
