add_library(object_recognition_core_common SHARED object_info.cpp)

target_link_libraries(object_recognition_core_common
  ${OpenCV_LIBS}
  object_recognition_core_db
)

install(TARGETS object_recognition_core_common
  DESTINATION lib
  )
