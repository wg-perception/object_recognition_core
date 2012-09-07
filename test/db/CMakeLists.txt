# test some Python interfaces
object_recognition_core_pytest(db_test)

if (NOT GTEST_FOUND)
  return()
endif()

# Testing core functionalities
if (ROS_GROOVY_FOUND)
catkin_add_gtest(or-db-test main.cpp
                     db_test.cpp
)
else()
add_gtest(or-db-test main.cpp
                     db_test.cpp
)
endif()
add_dependencies(or-db-test object_recognition_core_db)
target_link_libraries(or-db-test object_recognition_core_db)
