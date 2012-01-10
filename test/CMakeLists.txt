object_recognition_pytest(test_import)

object_recognition_pytest(test_common_io_ros_sink)

object_recognition_pytest(test_common_io_source_interface)
object_recognition_pytest(test_common_io_source_creation)

# check that at least the help completes correctly
object_recognition_pytest(test_help)

object_recognition_desktop_test(view_observations)

add_subdirectory(db)

# install some files so that they can be used by other projects when using the macros
file(COPY run_test.sh desktop_test.sh test_detection_pipeline.py test_training_pipeline.py DESTINATION ${CMAKE_BINARY_DIR}/share/test/)
