add_subdirectory(ros)

find_pcl_package()

include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

# deal with the basci io cells
ectomodule(io
           module.cpp
           csv.cpp
           GuessCsvWriter.cpp
           GuessTerminalWriter.cpp
)

link_ecto(io
          ${Boost_LIBRARIES}
          ${OpenCV_LIBRARIES}
          ${PCL_LIBRARIES}
          object_recognition_core_common
)

install_ecto_module(io)

# deal with the basic voter cells
ectomodule(voter
           Aggregator.cpp
           module_voter.cpp
)

link_ecto(voter
          ${Boost_LIBRARIES}
          ${OpenCV_LIBS}
          ${PCL_LIBRARIES}
          object_recognition_core_db
)

install_ecto_module(voter)
