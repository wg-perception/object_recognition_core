# install some files so that they can be used by other projects when using the macros
set(TEST_FILES run_test.sh
               desktop_test.sh
               test_config.py
               test_help.py
               test_sink.py
               test_source.py
)

file(COPY ${TEST_FILES}
     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test)

install(PROGRAMS ${TEST_FILES}
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
