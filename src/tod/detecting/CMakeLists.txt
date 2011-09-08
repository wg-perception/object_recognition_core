# deal with the detection TOD module
ectomodule(tod_detection
  DescriptorMatcher.cpp
  GuessGenerator.cpp
  module.cpp
  )

link_ecto(tod_detection
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)

install_ecto_module(tod_detection)
