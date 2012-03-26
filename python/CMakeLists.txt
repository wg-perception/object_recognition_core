configure_file(${CMAKE_CURRENT_SOURCE_DIR}/object_recognition_core/__init__.py
               ${python_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
               @ONLY
)
configure_file(__init__.py.plain.in
               ${python_LIBRARY_OUTPUT_DIRECTORY}/boost/__init__.py
               @ONLY
)
configure_file(__init__.py.plain.in
               ${python_LIBRARY_OUTPUT_DIRECTORY}/ecto_cells/__init__.py
               @ONLY
)

install(FILES ${python_LIBRARY_OUTPUT_DIRECTORY}/boost/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_core/boost
)
install(FILES ${python_LIBRARY_OUTPUT_DIRECTORY}/ecto_cells/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_core/ecto_cells
)

#install targets for all things python
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/object_recognition_core
        DESTINATION ${PYTHON_PACKAGES_PATH}
        COMPONENT main
        FILES_MATCHING PATTERN "*.py"
)

# override the init with a plain one
install(FILES ${python_LIBRARY_OUTPUT_DIRECTORY}/ecto_cells/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_core/
)
