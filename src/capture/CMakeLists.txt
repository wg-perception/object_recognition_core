include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#deal with the objcog module
ectomodule(capture
    capture_db.cpp
    )
link_ecto(capture
    objcog_db
    ${OpenCV_LIBS}
    )
install_ecto_module(capture)
