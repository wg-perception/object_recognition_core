include_directories(${CMAKE_CURRENT_SOURCE_DIR})
# deal with the tod module bag_reader.cpp guess_generator.cpp willow_tod
ectomodule(tod
                tod.cpp 
                #bag_reader.cpp
                twoDToThreeD.cpp
                cameraToWorld.cpp
                detecting/guess_generator.cpp
                detecting/guess_writer.cpp
                detecting/descriptor_matcher.cpp
                )
include_directories(${PCL_INCLUDE_DIRS})
link_ecto(tod ${OpenCV_LIBS} objcog_db ${PCL_LIBRARIES})
install_ecto_module(tod)

#deal with the objcog module
ectomodule(tod_db
    objcog_db.cpp
    )
link_ecto(tod_db
    objcog_db
    ${OpenCV_LIBS}
    )
install_ecto_module(tod_db)
