# deal with the detection TOD module
ectomodule(filters
  depth_filter.cpp
  module.cpp
  )

link_ecto(filters
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
)

install_ecto_module(filters)
