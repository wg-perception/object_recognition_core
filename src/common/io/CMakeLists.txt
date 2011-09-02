add_subdirectory(ros)

# deal with the detection TOD module
ectomodule(io
  module.cpp
  csv.cpp
  GuessCsvWriter.cpp
  )

link_ecto(io
  ${OpenCV_LIBS}
  ${pcl_LIBRARIES}
  ${Boost_LIBRARIES}
  )

install_ecto_module(io)
