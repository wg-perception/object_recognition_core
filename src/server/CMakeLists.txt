install(PROGRAMS server.py client.py
        DESTINATION share/${PROJECT_NAME}/bin
)

install(DIRECTORY launch conf
        DESTINATION share/${PROJECT_NAME}
)
