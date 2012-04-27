find_package(Boost REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories(SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${PYTHON_INCLUDE_PATH}
)

add_library(object_recognition_core_common SHARED
            dict_json_conversion.cpp
            object_info.cpp
)

target_link_libraries(object_recognition_core_common
                      object_recognition_core_db
                      ${Boost_LIBRARIES}
)


set_target_properties(object_recognition_core_common PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

install(TARGETS object_recognition_core_common
        DESTINATION lib
)
