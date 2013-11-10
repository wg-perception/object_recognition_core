set(SETUP_DOT_SH ${CATKIN_DEVEL_PREFIX}/setup.sh)

if (object_recognition_core_SOURCE_DIR)
get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(object_recognition_core_TEST_DIR ${SELF_DIR}/../test)
else()
  get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
  set(object_recognition_core_TEST_DIR ${SELF_DIR}/../test)
endif()

#testing macros
macro(object_recognition_core_pytest pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(${PROJECT_NAME}_${pyfile}.py
    ${object_recognition_core_TEST_DIR}/run_test.sh ${SETUP_DOT_SH} ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
    )
endmacro()

macro(object_recognition_core_desktop_test pyfile)
  if(DESKTOP_TEST)
    string(REPLACE ";" " " args "${ARGN}")
    add_test(desktop_${PROJECT_NAME}_${pyfile}.py
      ${object_recognition_core_TEST_DIR}/run_test.sh ${SETUP_DOT_SH} ${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}.py ${args}
      )
  endif(DESKTOP_TEST)
endmacro()

macro(object_recognition_core_data_download PATH_VAR DATA_FILE)
  set(data_base_url http://vault.willowgarage.com/wgdata1/vol1/ecto_data)
  set(${PATH_VAR} ${PROJECT_BINARY_DIR}/data/${DATA_FILE})
  if(NOT EXISTS ${${PATH_VAR}})
    message(STATUS "Data fetch.\n** Downloading:\n** ${data_base_url}/${DATA_FILE}\n** to:\n** ${${PATH_VAR}}")
    file(DOWNLOAD ${data_base_url}/${DATA_FILE} ${${PATH_VAR}})
  endif()
endmacro()

# macro testing an ORK configuration file
macro(object_recognition_core_config_test config_file)
  add_test(${PROJECT_NAME}_${config_file}
    ${object_recognition_core_TEST_DIR}/run_test.sh ${SETUP_DOT_SH} "${object_recognition_core_TEST_DIR}/test_config.py -c ${config_file}"
    )
endmacro()

# macro testing a script with --help: "script" must contain the whole path
macro(object_recognition_core_help_test script)
  add_test(${PROJECT_NAME}_help_${script}
    ${object_recognition_core_TEST_DIR}/run_test.sh ${SETUP_DOT_SH} "${object_recognition_core_TEST_DIR}/test_help.py ${script}"
    )
endmacro()

# macro testing a Source
# the first argument is the value returned by type_name
# the second argument is the Python module in which to find that source
# the third argument is a string of the dict used for parameters: has to be set to "{}" if none
macro(object_recognition_core_source_test source_name package_name parameters)
  add_test(${PROJECT_NAME}_source_${source_name}
      ${object_recognition_core_TEST_DIR}/run_test.sh ${SETUP_DOT_SH} "${object_recognition_core_TEST_DIR}/test_source.py ${source_name} ${package_name} ${parameters}"
  )
endmacro()

# macro testing a Sink
# the first argument is the value returned by type_name
# the second argument is the Python module in which to find that sink
# the third argument is a string of the dict used for parameters: has to be set to "{}" if none
macro(object_recognition_core_sink_test sink_name package_name parameters)
  add_test(${PROJECT_NAME}_sink_${sink_name}
    ${object_recognition_core_TEST_DIR}/run_test.sh ${SETUP_DOT_SH} "${object_recognition_core_TEST_DIR}/test_sink.py ${sink_name} ${package_name} ${parameters}"
  )
endmacro()
