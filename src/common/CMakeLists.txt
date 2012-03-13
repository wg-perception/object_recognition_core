add_library(object_recognition_core_common SHARED object_info.cpp)

target_link_libraries(object_recognition_core_common
  ${OpenCV_LIBS}
  object_recognition_core_db
)

set_target_properties(object_recognition_core_common PROPERTIES
                      VERSION ${object_recognition_core_VERSION}
                      SOVERSION ${object_recognition_core_SOVERSION}
)

install(TARGETS object_recognition_core_common
  DESTINATION lib
  )
