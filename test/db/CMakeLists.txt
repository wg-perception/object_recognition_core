# test some Python interfaces
object_recognition_core_pytest(db_test)

# TODO reenable but only locally so that the test does not fail on the farm
return()
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
