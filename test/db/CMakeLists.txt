# test some Python interfaces
include(${CMAKE_BINARY_DIR}/cmake/object_recognition_core/test.cmake)

object_recognition_core_pytest(db_test)

# Testing core functionalities
find_package(GTest)

if(NOT GTEST_FOUND)
  message(STATUS "Disabling testing since gtest was not found.")
  return()
endif()

include_directories(${GTEST_INCLUDE_DIRS})

add_executable(or-db-test
  main.cpp
  db_test.cpp
  )

target_link_libraries(or-db-test
  ${GTEST_LIBRARIES}
  object_recognition_core_db
  )

add_test(object_recognition_core_db_tests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/or-db-test)
