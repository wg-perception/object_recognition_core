cmake_minimum_required(VERSION 2.8)
project(sim)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif()

find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)
if(NOT VTK_USE_RENDERING)
  message(FATAL_ERROR "Example ${PROJECT_NAME} requires VTK_USE_RENDERING.")
endif()

include(${VTK_USE_FILE})

set( VTK_REQUIRED_LIBS vtkRendering  vtkGraphics vtkHybrid  vtkWidgets 
                     vtkImaging vtkIO vtkFiltering vtkCommon vtkVolumeRendering)

add_executable(plane_render plane_render.cpp)
target_link_libraries(plane_render ${VTK_REQUIRED_LIBS} ${OpenCV_LIBS} boost_thread)

find_package(ecto)
if(ecto_FOUND)
  #set the default path for built libraries to the "ecto_opencv" directory
  if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    ecto_python_env_gen(${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
  endif()

  ectomodule(sim
     module.cpp
     plane_render.cpp
  )
  link_ecto(sim
    ${VTK_REQUIRED_LIBS} ${OpenCV_LIBS}
  )
  set_target_properties(sim_ectomodule
    PROPERTIES
    COMPILE_DEFINITIONS "DO_ECTO"
    )
endif()
