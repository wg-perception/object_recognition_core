# deal with the training TOD modules
ectomodule(tod_training
    CameraToWorld.cpp
    KeypointsToMat.cpp
    ModelInserter.cpp
    ModelStacker.cpp
    module.cpp
    )

link_ecto(tod_training
  object_recognition_db
  ${OpenCV_LIBS}
  )

install_ecto_module(tod_training)
