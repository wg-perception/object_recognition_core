add_subdirectory(ros)

# deal with the basci io cells
ectomodule(io module.cpp
              csv.cpp
              GuessCsvWriter.cpp
              GuessTerminalWriter.cpp
              PipelineInfo.cpp
)

link_ecto(io ${OpenCV_LIBRARIES}
             object_recognition_core_common
)

install_ecto_module(io)

# deal with the basic voter cells
ectomodule(voter Aggregator.cpp
                 module_voter.cpp
)

link_ecto(voter ${OpenCV_LIBRARIES}
                object_recognition_core_db
)

install_ecto_module(voter)
