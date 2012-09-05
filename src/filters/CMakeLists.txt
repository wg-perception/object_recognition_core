# deal with the detection TOD module
ectomodule(filters DESTINATION object_recognition_core/ecto_cells
                   INSTALL
  depth_filter.cpp
  module.cpp
  )

link_ecto(filters
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)
