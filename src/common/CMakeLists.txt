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

if (ROS_GROOVY_OR_ABOVE_FOUND)
install(TARGETS object_recognition_core_common
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
else()
install(TARGETS object_recognition_core_common
        DESTINATION lib
)
endif()


# create a Python module to wrap the PoseResult
add_library(common_interface SHARED module_python.cpp
                                    wrap_db_pose_result.cpp
)

target_link_libraries(common_interface ${Boost_LIBRARIES}
                                       object_recognition_core_db
)


if (ROS_GROOVY_OR_ABOVE_FOUND)
  set(ECTO_PYTHON_BUILD_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/)
else()
  set(ECTO_PYTHON_BUILD_PATH ${CMAKE_BINARY_DIR}/gen/py/)                  
endif()


set_target_properties(common_interface PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${ECTO_PYTHON_BUILD_PATH}/${PROJECT_NAME}/boost
                                                  OUTPUT_NAME common
                                                  PREFIX ""
)

install(TARGETS common_interface
        DESTINATION ${ECTO_PYTHON_INSTALL_PATH}/${PROJECT_NAME}/boost
)
