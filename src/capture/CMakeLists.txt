include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#deal with the objcog module
ectomodule(capture
  module.cpp
  observations.cpp
  ObservationReader.cpp
  ObservationWriter.cpp
  DeltaRT.cpp
  RescaleRegisteredDepth.cpp
  )

link_ecto(capture
  object_recognition_db
  ${OpenCV_LIBS}
  )

install_ecto_module(capture)
