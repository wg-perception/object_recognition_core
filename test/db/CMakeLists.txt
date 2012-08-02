# test some Python interfaces
include(${CMAKE_BINARY_DIR}/cmake/object_recognition_core/test.cmake)

object_recognition_core_pytest(db_test)

# Testing core functionalities
add_gtest(or-db-test main.cpp
                     db_test.cpp
)
target_link_libraries(or-db-test object_recognition_core_db)
