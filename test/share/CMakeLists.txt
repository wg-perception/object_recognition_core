# install some files so that they can be used by other projects when using the macros
file(COPY
    run_test.sh
    desktop_test.sh
    test_detection_pipeline.py
    test_help.py
    test_sink.py
    test_source.py
    test_training_pipeline.py
    DESTINATION ${CMAKE_BINARY_DIR}/share/test/)
