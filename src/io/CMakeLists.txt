# deal with the basci io cells
ectomodule(io DESTINATION object_recognition_core/ecto_cells
              INSTALL
              module.cpp
              csv.cpp
              GuessCsvWriter.cpp
              GuessTerminalWriter.cpp
              PipelineInfo.cpp
)

link_ecto(io ${OpenCV_LIBRARIES}
             object_recognition_core_common
)

# deal with the basic voter cells
ectomodule(voter DESTINATION object_recognition_core/ecto_cells
                 INSTALL
                 Aggregator.cpp
                 module_voter.cpp
)

link_ecto(voter ${OpenCV_LIBRARIES}
                object_recognition_core_db
)
