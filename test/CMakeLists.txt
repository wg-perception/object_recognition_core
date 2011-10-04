#testing macros
macro(pytest pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(object_recognition_${pyfile}
    ${CMAKE_CURRENT_SOURCE_DIR}/run_test.sh ${CMAKE_BINARY_DIR}/python_path.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endmacro()

macro(desktop_test pyfile)
  if(DESKTOP_TEST)
    string(REPLACE ";" " " args "${ARGN}")
    add_test(desktop_object_recognition_${pyfile}
      ${CMAKE_CURRENT_SOURCE_DIR}/run_test.sh ${CMAKE_BINARY_DIR}/python_path.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
      )
  endif(DESKTOP_TEST)
endmacro()

macro(data_download PATH_VAR DATA_FILE)
  set(data_base_url http://vault.willowgarage.com/wgdata1/vol1/ecto_data)
  set(${PATH_VAR} ${PROJECT_BINARY_DIR}/data/${DATA_FILE})
  if(NOT EXISTS ${${PATH_VAR}})
    message(STATUS "Data fetch.\n** Downloading:\n** ${data_base_url}/${DATA_FILE}\n** to:\n** ${${PATH_VAR}}")
    file(DOWNLOAD ${data_base_url}/${DATA_FILE} ${${PATH_VAR}})
  endif()
endmacro()

pytest(test_import)

pytest(test_common_io_ros_source_kinect)
pytest(test_common_io_ros_source_bag_reader)
pytest(test_common_io_ros_sink)

pytest(test_common_io_source_interface)
pytest(test_common_io_source_creation)

#these check that atleast the help completes correctly
pytest(test_capture_help)
pytest(test_upload_help)
pytest(test_training_help)

desktop_test(view_observations)

add_subdirectory(db)
