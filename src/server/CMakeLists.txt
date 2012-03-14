cmake_minimum_required(VERSION 2.4.6)

rosbuild_lite_init()

if (ROS_ELECTRIC_FOUND)
    #find_ros_package(actionlib_msgs)
    #include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
    #genaction()
    #rosbuild_genmsg()
endif()
