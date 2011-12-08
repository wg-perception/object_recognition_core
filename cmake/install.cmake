set(prefix ${PROJECT_NAME}-${${PROJECT_NAME}_VERSION})
install(DIRECTORY ${CMAKE_PROJECT_SOURCE_DIR}/include/
  DESTINATION include/${prefix}
  COMPONENT main
  )

#install the unix_install
install(DIRECTORY ${CMAKE_BINARY_DIR}/share/
  DESTINATION share/${prefix}
  COMPONENT main
  )

