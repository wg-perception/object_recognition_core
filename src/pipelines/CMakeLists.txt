find_package(Eigen)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})

# module with a pipeline that returns a constant value
ectomodule(pipelines DESTINATION object_recognition_core/ecto_cells
                     INSTALL
                     module.cpp
                     ConstantPipeline.cpp
)

link_ecto(pipelines ${OpenCV_LIBRARIES}
                    object_recognition_core_common
)
