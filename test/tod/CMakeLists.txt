find_package(GTest)

if(NOT GTEST_FOUND)
  message(STATUS "Disabling testing since gtest was not found.")
  return()
endif()

include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${object_recognition_SOURCE_DIR}/src/tod/common)

add_executable(or-tod-test
  main.cpp
  test_maximum_clique.cpp
  )

target_link_libraries(or-tod-test
  ${GTEST_LIBRARIES}
  tod_detection_ectomodule
  )

add_test(object_recognition_tod_tests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/or-tod-test)
