include(${CMAKE_BINARY_DIR}/cmake/object_recognition_core/test.cmake)

# deal with the subdirectories as they contain macros
add_subdirectory(db)
add_subdirectory(share)

object_recognition_core_pytest(test_import)

object_recognition_core_desktop_test(view_observations)

# check that at least the help completes correctly
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/detection)
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/training)

# check the different sources
object_recognition_core_source_test(bag_reader "{'bag':'a.bag'}" "object_recognition_core.io")
object_recognition_core_source_test(ros_kinect "{}" "object_recognition_core.io")
object_recognition_core_source_test(openni "{}" "object_recognition_core.io")

# check the different sinks
object_recognition_core_sink_test(publisher "{}" "object_recognition_core.io")
