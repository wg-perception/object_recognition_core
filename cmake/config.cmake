set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)
set(${PROJECT_NAME}_LIBRARY_DIRS ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}) 
set(${PROJECT_NAME}_LIBRARIES
    object_recognition_db
    opencv_candidate
)

configure_file(cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  @ONLY
)

configure_file(cmake/${PROJECT_NAME}Config-version.cmake.in
  ${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config-version.cmake
  @ONLY
)

#this is a bit simple.
set(${PROJECT_NAME}_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include/${prefix})

set(${PROJECT_NAME}_LIBRARY_DIRS ${CMAKE_INSTALL_PREFIX}/lib) 
set(${PROJECT_NAME}_LIBRARIES
    object_recognition_db
    opencv_candidate
)

configure_file(cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_BINARY_DIR}/share/${PROJECT_NAME}Config.cmake
  @ONLY
)

configure_file(cmake/${PROJECT_NAME}Config-version.cmake.in
  ${CMAKE_BINARY_DIR}/share/${PROJECT_NAME}Config-version.cmake
  @ONLY
)
