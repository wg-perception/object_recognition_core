add_subdirectory(ros)

# deal with the basci io cells
ectomodule(io
  module.cpp
  csv.cpp
  GuessCsvWriter.cpp
  GuessTerminalWriter.cpp
  )

link_ecto(io
    object_recognition_core_common
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

install_ecto_module(io)

# deal with the basic voter cells
ectomodule(voter
           Aggregator.cpp
           module_voter.cpp
)

link_ecto(voter
          ${OpenCV_LIBS}
          object_recognition_core_db
          ${pcl_LIBRARIES}
          ${Boost_LIBRARIES}
)

set_target_properties(voter_ectomodule PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${ecto_module_PYTHON_INSTALL}/io
)

install(TARGETS voter_ectomodule
        DESTINATION ${ecto_module_PYTHON_INSTALL}/io
        COMPONENT main
)
