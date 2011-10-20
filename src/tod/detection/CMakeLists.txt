# deal with the detection TOD module


include_directories(${CMAKE_CURRENT_LIST_DIR}/../common)

ectomodule(tod_detection
  DescriptorMatcher.cpp
  GuessGenerator.cpp
  DescriptorLoader.cpp
  ModelReader.cpp
  module.cpp
  )

link_ecto(tod_detection
  ${OpenCV_LIBS}
  object_recognition_db
  ${pcl_LIBRARIES}
  ${Boost_LIBRARIES}
  opencv_candidate
)

install_ecto_module(tod_detection)
