find_ros_package(geometry_msgs)
find_ros_package(sensor_msgs)

ectomodule(conversion_ros
    module.cpp
    PointCloudMessageToDepthMessage.cpp
    )

link_ecto(conversion_ros
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${OpenCV_LIBS}
    )

install_ecto_module(conversion_ros)
