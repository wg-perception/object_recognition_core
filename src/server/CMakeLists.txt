if (ROS_GROOVY_OR_ABOVE_FOUND)
install(PROGRAMS server.py client.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 
install(DIRECTORY launch conf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
else()
install(PROGRAMS server.py client.py
        DESTINATION share/${PROJECT_NAME}/bin
)

install(DIRECTORY launch conf
        DESTINATION share/${PROJECT_NAME}
)
endif()
