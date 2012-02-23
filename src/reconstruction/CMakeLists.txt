include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_package(ecto_pcl REQUIRED)
find_package(Eigen REQUIRED)
find_package(image_pipeline REQUIRED)
find_package(PCL REQUIRED)

include_directories(SYSTEM
                    ${ecto_pcl_INCLUDE_DIRS}
                    ${EIGEN_INCLUDE_DIRS}
                    ${image_pipeline_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
)

#deal with the objcog module
ectomodule(reconstruction
  module.cpp
  #surfel_conversion.cpp
  #surfels.cpp
  reconstruction_db.cpp
  #SurfelReconstruction.cpp
  PointCloudTransform.cpp
  PointCloudAccumulator.cpp
  PointCloudMesh.cpp
  )

set_target_properties(reconstruction_ectomodule
      PROPERTIES
      COMPILE_FLAGS "-Wno-unknown-pragmas -Wno-unused-but-set-variable"
      )

link_ecto(reconstruction
    ${pcl_LIBRARIES}
    ${OpenCV_LIBS}
    object_recognition_core_db
)

install_ecto_module(reconstruction)
