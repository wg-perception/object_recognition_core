install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
        DESTINATION include/${PROJECT_NAME}
        COMPONENT main
)

#install the unix_install
install(DIRECTORY ${CMAKE_BINARY_DIR}/share/
        DESTINATION share/${PROJECT_NAME}
        COMPONENT main
)
