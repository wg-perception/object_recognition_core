configure_file(__init__.py.split.in
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

install(FILES ${python_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_core
)
install(FILES ${python_LIBRARY_OUTPUT_DIRECTORY}/boost/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_core/boost
)
install(FILES ${python_LIBRARY_OUTPUT_DIRECTORY}/ecto_cells/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_core/ecto_cells
)

