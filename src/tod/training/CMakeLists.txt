# deal with the training TOD modules
ectomodule(tod_training
    CameraToWorld.cpp
    FilteredModelStacker.cpp
    KeypointsToMat.cpp
    KeypointValidator.cpp
    ModelInserter.cpp
    ModelStacker.cpp
    module.cpp
    PointMerger.cpp
    PrepareForG2O.cpp
    )

link_ecto(tod_training
  object_recognition_db
  ${OpenCV_LIBS}
  opencv_candidate
  )

install_ecto_module(tod_training)
