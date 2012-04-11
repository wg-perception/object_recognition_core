# install the include folder
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
        DESTINATION include
        COMPONENT main
)

# install the share folder
install(DIRECTORY ${CMAKE_BINARY_DIR}/share/
        DESTINATION share/${PROJECT_NAME}
        COMPONENT main
        USE_SOURCE_PERMISSIONS
)
