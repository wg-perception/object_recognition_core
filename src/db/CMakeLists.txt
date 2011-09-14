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
  opencv.cpp
  utils.cpp
  )

target_link_libraries(object_recognition_db
  boost_system
  boost_serialization
  boost_thread
  boost_filesystem
  boost_regex
  ${CURL_LIBRARIES}
  ${OpenCV_LIBS}
  )

#install the main shared lib
install(TARGETS object_recognition_db
  DESTINATION lib
  COMPONENT main
  )

ectomodule(object_recognition_db
    module.cpp
    )
link_ecto(object_recognition_db
  object_recognition_db
  )