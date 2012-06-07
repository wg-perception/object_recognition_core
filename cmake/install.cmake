# install the include folder
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
        DESTINATION include
        COMPONENT main
)

# install the share folder
install(DIRECTORY ${CMAKE_BINARY_DIR}/share/test
        DESTINATION share/${PROJECT_NAME}
        COMPONENT main
        USE_SOURCE_PERMISSIONS
)

if (ROS_ELECTRIC_FOUND)
# install the applications
install(PROGRAMS ${PROJECT_SOURCE_DIR}/apps/detection
                 ${PROJECT_SOURCE_DIR}/apps/training 
        DESTINATION bin
)

install(FILES ${PROJECT_SOURCE_DIR}/apps/roscompat.py
        DESTINATION bin
)
else()
# install the applications
install(PROGRAMS ${PROJECT_SOURCE_DIR}/apps/detection
                 ${PROJECT_SOURCE_DIR}/apps/training
        DESTINATION share/${PROJECT_NAME}/bin
)
endif()
