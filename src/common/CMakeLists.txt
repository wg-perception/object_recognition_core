add_subdirectory(filters)
add_subdirectory(io)
add_subdirectory(voters)

add_library(object_recognition_common SHARED pose_result.cpp)

target_link_libraries(object_recognition_common
  ${OpenCV_LIBS}
)

set_target_properties(object_recognition_common PROPERTIES
  VERSION ${object_recognition_VERSION}
  SOVERSION ${object_recognition_SOVERSION}
  )

install(TARGETS object_recognition_common
  DESTINATION lib
  )
