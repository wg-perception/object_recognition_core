find_pcl_package()
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_library(object_recognition_core_common SHARED
            dict_json_conversion.cpp
            object_info.cpp
)

target_link_libraries(object_recognition_core_common
                      ${OpenCV_LIBRARIES}
                      ${PCL_LIBRARIES}
                      object_recognition_core_db
)


set_target_properties(object_recognition_core_common PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

install(TARGETS object_recognition_core_common
        DESTINATION lib
)
