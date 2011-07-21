find_package(Boost COMPONENTS
  system
  serialization
  thread
  filesystem
  regex
  REQUIRED
  )
  
add_subdirectory(json_spirit)

find_package(CURL REQUIRED)
find_package(OpenCV)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../tod )

add_library(objcog_db SHARED couch.cpp db.cpp db_couch.cpp serialization_opencv.cpp opencv.cpp)
target_link_libraries(objcog_db  ${Boost_LIBRARIES} ${CURL_LIBRARIES} ${OpenCV_LIBS} json_spirit)

set_target_properties(objcog_db PROPERTIES
            VERSION 0.1.1
            SOVERSION 0.1
            )
#install the main shared lib
install(TARGETS objcog_db
  DESTINATION lib
  COMPONENT main
  )
