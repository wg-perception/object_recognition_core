include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#deal with the objcog module
ectomodule(capture
    module.cpp
    capture_db.cpp
    ObservationReader.cpp
    ObservationWriter.cpp
    DeltaRT.cpp
    )
link_ecto(capture
    object_recognition_db
    ${OpenCV_LIBS}
    )
install_ecto_module(capture)
