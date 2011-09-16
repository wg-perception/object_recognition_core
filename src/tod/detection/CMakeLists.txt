# deal with the detection TOD module
ectomodule(tod_detection
  DescriptorMatcher.cpp
  GuessGenerator.cpp
  ModelReader.cpp
  module.cpp
  impl/maximum_clique.cpp
  )

link_ecto(tod_detection
  ${OpenCV_LIBS}
  object_recognition_db
  ${pcl_LIBRARIES}
  ${Boost_LIBRARIES}
  opencv_candidate
)

install_ecto_module(tod_detection)
