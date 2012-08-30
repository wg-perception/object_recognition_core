# test some Python interfaces
object_recognition_core_pytest(db_test)

# Testing core functionalities
add_gtest(or-db-test main.cpp
                     db_test.cpp
)
if (NOT GTEST_FOUND)
  return()
endif()
add_dependencies(or-db-test object_recognition_core_db)
target_link_libraries(or-db-test object_recognition_core_db)
