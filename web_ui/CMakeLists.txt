# install the web interface
install(DIRECTORY _attachments
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/web_ui/
)
install(FILES _id
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/web_ui/
)
install(PROGRAMS push.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/web_ui/
)

# create a target to install a web interface
find_program(COUCHAPP couchapp)

if(COUCHAPP-NOTFOUND)
    message(STATUS "In order to install the webui, please install couchapp.")
    message(STATUS "sudo pip install -U couchapp")
    return()
endif()

add_custom_target(or_web_ui)

set(OR_WEB_UI_LOCATION http://localhost:5984/or_web_ui CACHE STRING "The location for the web ui to be installed to.")
add_custom_command(TARGET or_web_ui
    COMMAND ${COUCHAPP} push ${CMAKE_CURRENT_SOURCE_DIR} ${OR_WEB_UI_LOCATION}
)
