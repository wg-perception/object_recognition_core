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
                                                 ${catkin_LIBRARIES}
                                                 ${CURL_LIBRARIES}
                                                 ${OpenCV_LIBRARIES}
)

install(TARGETS object_recognition_core_db
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
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
                      LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/boost
                      OUTPUT_NAME interface
                      PREFIX ""
)

install(TARGETS db_interface
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/boost
)

# Create the ecto module with a few cells
ectomodule(db DESTINATION object_recognition_core/ecto_cells
              INSTALL
           module.cpp
           ModelWriter.cpp
           ObservationReader.cpp
           ObservationWriter.cpp
)

link_ecto(db
          object_recognition_core_db
)
