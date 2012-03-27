configure_file(__init__.py.plain.in
               ${ecto_module_PYTHON_OUTPUT}/../boost/__init__.py
               @ONLY
)
configure_file(__init__.py.plain.in
               ${ecto_module_PYTHON_OUTPUT}/__init__.py
               @ONLY
)

install(FILES ${ecto_module_PYTHON_OUTPUT}/../boost/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_core/boost
)
install(FILES ${ecto_module_PYTHON_OUTPUT}/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_core/ecto_cells
)
