find_package(Boost COMPONENTS
  system
  serialization
  thread
  filesystem
  regex
  REQUIRED
  )

find_package(CURL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# create a shared library to deal with the different DB components
add_library(object_recognition_core_db
            SHARED
            db.cpp
            db_couch.cpp
            db_filesystem.cpp
            opencv.cpp
            ../../include/object_recognition_core/common/json_spirit/json_spirit_reader.cpp
            ../../include/object_recognition_core/common/json_spirit/json_spirit_value.cpp
            ../../include/object_recognition_core/common/json_spirit/json_spirit_writer.cpp
            model_utils.cpp
            observations.cpp
            view.cpp
)

target_link_libraries(object_recognition_core_db ${Boost_LIBRARIES}
                                                 ${CURL_LIBRARIES}
                                                 ${OpenCV_LIBS}
                                                 ${OpenCV_LIBRARIES}
                                                 ${ecto_LIBRARIES}
)

set_target_properties(object_recognition_core_db PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

install(TARGETS object_recognition_core_db
        DESTINATION lib
)

# create a Python module to wrap the DB functionalities
add_library(db_interface SHARED
            module_python.cpp
            wrap_db_parameters.cpp
            wrap_db_documents.cpp
            wrap_object_db.cpp
)

target_link_libraries(db_interface
                      object_recognition_core_common
                      object_recognition_core_db
)

set_target_properties(db_interface PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${ecto_module_PYTHON_OUTPUT}/../boost
                      OUTPUT_NAME interface
                      PREFIX ""
)

install(TARGETS db_interface
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_core/boost
)

# Create the ecto module with a few cells
ectomodule(db
           module.cpp
           ModelWriter.cpp
           ObservationReader.cpp
           ObservationWriter.cpp
)

link_ecto(db
          object_recognition_core_db
)

install_ecto_module(db)
