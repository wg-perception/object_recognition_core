add_subdirectory(ros)

# deal with the detection TOD module
ectomodule(io
  module.cpp
  csv.cpp
  GuessCsvWriter.cpp
  GuessTerminalWriter.cpp
  )

link_ecto(io
  object_recognition_common
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  )

install_ecto_module(io)
