if (ROS_GROOVY_FOUND)
include(${CATKIN_BUILD_PREFIX}/share/object_recognition_core/cmake/test.cmake)
else()
include(${CMAKE_BINARY_DIR}/cmake/object_recognition_core/test.cmake)
endif()

# deal with the subdirectories as they contain macros
add_subdirectory(db)
add_subdirectory(share)

object_recognition_core_pytest(test_import)

object_recognition_core_desktop_test(view_observations)

# check that at least the help completes correctly
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/detection)
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/training)
#object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../src/server/server.py)
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../src/server/client.py)

# check the different sources
object_recognition_core_source_test(bag_reader "{'bag':'a.bag'}" "object_recognition_core.io")
object_recognition_core_source_test(ros_kinect "{}" "object_recognition_core.io")
object_recognition_core_source_test(openni "{}" "object_recognition_core.io")

# check the different sinks
object_recognition_core_sink_test(publisher "{}" "object_recognition_core.io")

find_package(object_recognition_tod QUIET)
if (object_recognition_tod_FOUND)
  object_recognition_core_detection_test(${CMAKE_CURRENT_SOURCE_DIR}/../src/server/conf/config_detection.sample)
endif()
