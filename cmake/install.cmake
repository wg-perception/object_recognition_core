# install the include folder
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/object_recognition_core/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        COMPONENT main
)

# install the applications
install(PROGRAMS ${PROJECT_SOURCE_DIR}/apps/detection
                 ${PROJECT_SOURCE_DIR}/apps/training
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install the DB scripts
install(DIRECTORY ${PROJECT_SOURCE_DIR}/apps/dbscripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
        USE_SOURCE_PERMISSIONS
)
