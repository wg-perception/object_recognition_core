# install some files so that they can be used by other projects when using the macros
set(TEST_FILES run_test.sh
               desktop_test.sh
               test_detection_pipeline.py
               test_help.py
               test_sink.py
               test_source.py
               test_training_pipeline.py
)

if (ROS_GROOVY_OR_ABOVE_FOUND)
file(COPY ${TEST_FILES}
     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test)

install(PROGRAMS ${TEST_FILES}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
else()
file(COPY ${TEST_FILES} DESTINATION ${CMAKE_BINARY_DIR}/share/test/)
install(PROGRAMS ${TEST_FILES}
        DESTINATION share/object_recognition_core/test
)
endif()
