#testing macros
macro(object_recognition_pytest pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(${PROJECT_NAME}_${pyfile}
    ${object_recognition_SHARE_DIRS}/test/run_test.sh ${CMAKE_BINARY_DIR}/python_path.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endmacro()

macro(object_recognition_desktop_test pyfile)
  if(DESKTOP_TEST)
    string(REPLACE ";" " " args "${ARGN}")
    add_test(desktop_${PROJECT_NAME}_${pyfile}
      ${object_recognition_SHARE_DIRS}/test/run_test.sh ${CMAKE_BINARY_DIR}/python_path.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
      )
  endif(DESKTOP_TEST)
endmacro()

macro(object_recognition_data_download PATH_VAR DATA_FILE)
  set(data_base_url http://vault.willowgarage.com/wgdata1/vol1/ecto_data)
  set(${PATH_VAR} ${PROJECT_BINARY_DIR}/data/${DATA_FILE})
  if(NOT EXISTS ${${PATH_VAR}})
    message(STATUS "Data fetch.\n** Downloading:\n** ${data_base_url}/${DATA_FILE}\n** to:\n** ${${PATH_VAR}}")
    file(DOWNLOAD ${data_base_url}/${DATA_FILE} ${${PATH_VAR}})
  endif()
endmacro()

# macro testing a detection pipeline
macro(object_recognition_detection_test config_file)
  add_test(${PROJECT_NAME}_detection
    ${object_recognition_SHARE_DIRS}/test/run_test.sh ${CMAKE_BINARY_DIR}/python_path.sh "${object_recognition_SHARE_DIRS}/test/test_detection_pipeline.py -c ${config_file} --node_name \"\""
    )
endmacro()

# macro testing a training pipeline
macro(object_recognition_training_test config_file)
  add_test(${PROJECT_NAME}_training
    ${object_recognition_SHARE_DIRS}/test/run_test.sh ${CMAKE_BINARY_DIR}/python_path.sh "${object_recognition_SHARE_DIRS}/test/test_training_pipeline.py -c ${config_file} --node_name \"\""
    )
endmacro()

# macro testing a script with --help: "script" must contain the whole path
macro(object_recognition_help_test script)
  add_test(${PROJECT_NAME}_help_script
    ${object_recognition_SHARE_DIRS}/test/run_test.sh ${CMAKE_BINARY_DIR}/python_path.sh "${object_recognition_SHARE_DIRS}/test/test_help.py ${script}"
    )
endmacro()
