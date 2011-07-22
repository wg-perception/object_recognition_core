include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# deal with the TOD module
ectomodule(tod
                tod.cpp 
                twoDToThreeD.cpp
                cameraToWorld.cpp
                detecting/guess_generator.cpp
                detecting/guess_csv_writer.cpp
                detecting/descriptor_matcher.cpp
                )
include_directories(${PCL_INCLUDE_DIRS})
link_ecto(tod ${OpenCV_LIBS} object_recognition_db ${PCL_LIBRARIES} boost_program_options)
install_ecto_module(tod)

#deal with the db part of TOD
ectomodule(tod_db
    objcog_db.cpp
    )
link_ecto(tod_db object_recognition_db ${OpenCV_LIBS})
install_ecto_module(tod_db)
