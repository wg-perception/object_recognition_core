find_package(ecto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set_ecto_install_package_name(ecto_tod)

# deal with the tod module bag_reader.cpp guess_generator.cpp willow_tod
ectorosmodule(tod tod.cpp twoDToThreeD.cpp cameraToWorld.cpp)
link_ecto(tod ${OpenCV_LIBS})
install_ecto_module(tod)

#deal with the objecog module
ectomodule(objcog_db objcog_db.cpp)
link_ecto(objcog_db ${OpenCV_LIBS} ${objcog_db_LIBRARIES})
install_ecto_module(objcog_db)
