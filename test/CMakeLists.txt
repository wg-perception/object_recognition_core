# deal with the subdirectories as they contain macros
add_subdirectory(db)
add_subdirectory(share)


object_recognition_pytest(test_import)

object_recognition_pytest(test_common_io_ros_sink)

object_recognition_pytest(test_common_io_source_interface)
object_recognition_pytest(test_common_io_source_creation)

object_recognition_desktop_test(view_observations)

# check that at least the help completes correctly
object_recognition_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/detection)
object_recognition_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/mesh_object)
object_recognition_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/training)
object_recognition_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/upload)
