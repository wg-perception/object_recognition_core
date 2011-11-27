# deal with the training TOD modules
ectomodule(tod_training
    CameraToWorld.cpp
    KeypointValidator.cpp
    ModelStacker.cpp
    ModelFiller.cpp
    module.cpp
    PointMerger.cpp
    PrepareForG2O.cpp
    )

link_ecto(tod_training
  object_recognition_db
  ${OpenCV_LIBS}
  opencv_candidate
  ${pcl_LIBRARIES}
  tod_common
  )

install_ecto_module(tod_training)
