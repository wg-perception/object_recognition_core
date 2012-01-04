find_package(Boost COMPONENTS
  system
  serialization
  thread
  filesystem
  regex
  REQUIRED
  )

find_package(CURL REQUIRED)
find_package(OpenCV)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  )

add_library(object_recognition_db SHARED
  db.cpp
  db_couch.cpp
  db_filesystem.cpp
  opencv.cpp
  ../../include/object_recognition/common/json_spirit/json_spirit_reader.cpp
  ../../include/object_recognition/common/json_spirit/json_spirit_value.cpp
  ../../include/object_recognition/common/json_spirit/json_spirit_writer.cpp
  model_utils.cpp
  observations.cpp
  view_types.cpp
  )

target_link_libraries(object_recognition_db
  boost_filesystem
  boost_regex
  boost_system
  boost_serialization
  boost_thread
  ${CURL_LIBRARIES}
  opencv_core
  opencv_highgui
  ecto
  )


set_target_properties(object_recognition_db PROPERTIES
  VERSION ${object_recognition_VERSION}
  SOVERSION ${object_recognition_SOVERSION}
  )
#install the main shared lib
install(TARGETS object_recognition_db
  DESTINATION lib
  )

ectomodule(object_recognition_db
    module.cpp
    wrap_db_parameters.cpp
    wrap_db_documents.cpp
    ModelWriter.cpp
    ObservationReader.cpp
    ObservationWriter.cpp
    )

link_ecto(object_recognition_db
  object_recognition_db
  )

install_ecto_module(object_recognition_db)
