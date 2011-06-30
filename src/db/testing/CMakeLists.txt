ENABLE_TESTING()
find_package(OpenCV REQUIRED)

add_executable( couch_surfing couch_surfing.cpp)
target_link_libraries(couch_surfing objcog_db ${OpenCV_LIBS})

file(COPY lena.bmp DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_test(couch_test ${EXECUTABLE_OUTPUT_PATH}/couch_surfing)
