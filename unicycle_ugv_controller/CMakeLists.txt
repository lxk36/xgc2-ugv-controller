cmake_minimum_required(VERSION 3.0.2)
project(unicycle_ugv_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rigid_state_estimator_msgs
  roscpp
  ros1_utils
  std_msgs
  unicycle_reference_trajectory_msgs
)
find_package(Eigen3 REQUIRED)
find_package(xgc2_acados REQUIRED CONFIG)
find_package(xgc2_state_machine REQUIRED CONFIG)
find_package(xgc2_math REQUIRED CONFIG)

if(COMMAND xgc2_acados_require)
  xgc2_acados_require()
endif()

set(UGV_ACADOS_INCLUDE_DIRS ${XGC2_ACADOS_INCLUDE_DIRS})
set(UGV_ACADOS_LIBRARIES ${XGC2_ACADOS_LIBRARIES})
set(UGV_ACADOS_LIBRARY_DIRS ${XGC2_ACADOS_LIBRARY_DIRS})
set(UGV_ACADOS_PYTHONPATH ${XGC2_ACADOS_PYTHONPATH} ${XGC2_ACADOS_PYTHON_PATH})
set(UGV_ACADOS_SOURCE_DIR "${XGC2_ACADOS_SOURCE_DIR}")
set(UGV_ACADOS_INSTALL_DIR "${XGC2_ACADOS_INSTALL_DIR}")
if(NOT UGV_ACADOS_SOURCE_DIR AND EXISTS "/opt/xgc2/acados/interfaces/acados_template")
  set(UGV_ACADOS_SOURCE_DIR "/opt/xgc2/acados")
endif()
if(NOT UGV_ACADOS_INSTALL_DIR AND EXISTS "/opt/xgc2/acados/lib")
  set(UGV_ACADOS_INSTALL_DIR "/opt/xgc2/acados")
endif()
if(UGV_ACADOS_SOURCE_DIR)
  list(APPEND UGV_ACADOS_INCLUDE_DIRS
    "${UGV_ACADOS_SOURCE_DIR}"
    "${UGV_ACADOS_SOURCE_DIR}/interfaces"
    "${UGV_ACADOS_SOURCE_DIR}/interfaces/acados_template")
  list(APPEND UGV_ACADOS_PYTHONPATH "${UGV_ACADOS_SOURCE_DIR}/interfaces/acados_template")
endif()
if(UGV_ACADOS_INSTALL_DIR)
  list(APPEND UGV_ACADOS_INCLUDE_DIRS
    "${UGV_ACADOS_INSTALL_DIR}/include"
    "${UGV_ACADOS_INSTALL_DIR}/include/hpipm/include"
    "${UGV_ACADOS_INSTALL_DIR}/include/blasfeo/include")
  list(APPEND UGV_ACADOS_LIBRARY_DIRS "${UGV_ACADOS_INSTALL_DIR}/lib")
endif()
if(UGV_ACADOS_INCLUDE_DIRS)
  list(REMOVE_DUPLICATES UGV_ACADOS_INCLUDE_DIRS)
endif()
if(UGV_ACADOS_LIBRARY_DIRS)
  list(REMOVE_DUPLICATES UGV_ACADOS_LIBRARY_DIRS)
endif()
if(UGV_ACADOS_PYTHONPATH)
  list(REMOVE_DUPLICATES UGV_ACADOS_PYTHONPATH)
endif()
string(REPLACE ";" ":" UGV_ACADOS_LIBRARY_PATH_ENV "${UGV_ACADOS_LIBRARY_DIRS}")
string(REPLACE ";" ":" UGV_ACADOS_PYTHONPATH_ENV "${UGV_ACADOS_PYTHONPATH}")

set(UGV_NMPC_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated/nmpc/unicycle_nmpc")
set(UGV_NMPC_TOOL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/tools")
set(UGV_NMPC_SOLVER_C "${UGV_NMPC_GENERATED_DIR}/acados_solver_unicycle_nmpc.c")
set(UGV_NMPC_SOLVER_H "${UGV_NMPC_GENERATED_DIR}/acados_solver_unicycle_nmpc.h")
set(UGV_NMPC_HORIZON "1.0")
set(UGV_NMPC_STEPS "10")
set(UGV_NMPC_GENERATOR_INPUTS
  "${UGV_NMPC_TOOL_PATH}/unicycle_nmpc/controller.py"
  "${UGV_NMPC_TOOL_PATH}/unicycle_nmpc/export_ros_generated_code.py")

set(UGV_NMPC_REGENERATE FALSE)
if(NOT EXISTS "${UGV_NMPC_SOLVER_C}" OR NOT EXISTS "${UGV_NMPC_SOLVER_H}")
  set(UGV_NMPC_REGENERATE TRUE)
else()
  file(READ "${UGV_NMPC_SOLVER_H}" UGV_NMPC_SOLVER_HEADER)
  string(FIND "${UGV_NMPC_SOLVER_HEADER}" "#define UNICYCLE_NMPC_N      ${UGV_NMPC_STEPS}" UGV_NMPC_STEPS_MATCH)
  string(FIND "${UGV_NMPC_SOLVER_HEADER}" "#define UNICYCLE_NMPC_NX     5" UGV_NMPC_NX_MATCH)
  string(FIND "${UGV_NMPC_SOLVER_HEADER}" "#define UNICYCLE_NMPC_NU     2" UGV_NMPC_NU_MATCH)
  if(UGV_NMPC_STEPS_MATCH EQUAL -1 OR UGV_NMPC_NX_MATCH EQUAL -1 OR UGV_NMPC_NU_MATCH EQUAL -1)
    set(UGV_NMPC_REGENERATE TRUE)
  endif()
  foreach(_input IN LISTS UGV_NMPC_GENERATOR_INPUTS)
    if("${_input}" IS_NEWER_THAN "${UGV_NMPC_SOLVER_C}")
      set(UGV_NMPC_REGENERATE TRUE)
    endif()
  endforeach()
endif()

if(UGV_NMPC_REGENERATE)
  find_program(PYTHON3_EXECUTABLE python3)
  if(NOT PYTHON3_EXECUTABLE)
    message(FATAL_ERROR "python3 is required to generate UGV NMPC acados code")
  endif()
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E env
      ACADOS_SOURCE_DIR=${UGV_ACADOS_SOURCE_DIR}
      ACADOS_ROOT=${UGV_ACADOS_SOURCE_DIR}
      XGC2_ACADOS_SOURCE_DIR=${UGV_ACADOS_SOURCE_DIR}
      XGC2_ACADOS_INSTALL_DIR=${UGV_ACADOS_INSTALL_DIR}
      XGC2_ACADOS_LIBRARY_DIRS=${UGV_ACADOS_LIBRARY_PATH_ENV}
      XGC2_ACADOS_PYTHONPATH=${UGV_ACADOS_PYTHONPATH_ENV}
      LD_LIBRARY_PATH=${UGV_ACADOS_LIBRARY_PATH_ENV}:$ENV{LD_LIBRARY_PATH}
      MPLBACKEND=Agg
      PYTHONPATH=${UGV_NMPC_TOOL_PATH}:${UGV_ACADOS_PYTHONPATH_ENV}:$ENV{PYTHONPATH}
      ${PYTHON3_EXECUTABLE} -B -m unicycle_nmpc.export_ros_generated_code
      --output-dir ${UGV_NMPC_GENERATED_DIR}
      --horizon ${UGV_NMPC_HORIZON}
      --steps ${UGV_NMPC_STEPS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE UGV_NMPC_EXPORT_STATUS)
  if(NOT UGV_NMPC_EXPORT_STATUS EQUAL 0)
    message(FATAL_ERROR "failed to generate UGV NMPC acados code")
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES unicycle_ugv_controller_nmpc_runtime
  CATKIN_DEPENDS geometry_msgs nav_msgs rigid_state_estimator_msgs roscpp ros1_utils std_msgs
                 unicycle_reference_trajectory_msgs
  CFG_EXTRAS unicycle_ugv_controller-dependencies.cmake
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${UGV_NMPC_GENERATED_DIR}
  ${UGV_ACADOS_INCLUDE_DIRS}
)

file(GLOB UGV_NMPC_MODEL_SOURCES "${UGV_NMPC_GENERATED_DIR}/unicycle_nmpc_model/*.c")
file(GLOB UGV_NMPC_COST_SOURCES "${UGV_NMPC_GENERATED_DIR}/unicycle_nmpc_cost/*.c")
set(UGV_NMPC_GENERATED_SOURCES
  ${UGV_NMPC_SOLVER_C}
  ${UGV_NMPC_MODEL_SOURCES}
  ${UGV_NMPC_COST_SOURCES})

add_library(unicycle_ugv_controller_nmpc_generated STATIC ${UGV_NMPC_GENERATED_SOURCES})
target_include_directories(unicycle_ugv_controller_nmpc_generated PUBLIC
  ${UGV_NMPC_GENERATED_DIR}
  ${UGV_ACADOS_INCLUDE_DIRS})
target_link_libraries(unicycle_ugv_controller_nmpc_generated ${UGV_ACADOS_LIBRARIES} m)
set_target_properties(unicycle_ugv_controller_nmpc_generated PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(unicycle_ugv_controller_nmpc_runtime
  src/common_types.cpp
  src/nmpc/unicycle_nmpc_solver.cpp
  src/nmpc/nmpc_tracking_backend.cpp)
add_dependencies(unicycle_ugv_controller_nmpc_runtime unicycle_ugv_controller_nmpc_generated ${catkin_EXPORTED_TARGETS})
target_link_libraries(unicycle_ugv_controller_nmpc_runtime
  unicycle_ugv_controller_nmpc_generated
  xgc2_math::control
  ${catkin_LIBRARIES}
  "-Wl,--no-as-needed"
  ${UGV_ACADOS_LIBRARIES}
  "-Wl,--as-needed"
  m)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/reference_cache.cpp
  src/unicycle_ugv_controller.cpp
  src/unicycle_ugv_ros_node.cpp
  src/input/command_input_producer.cpp
  src/input/pva_reference_input_producer.cpp
  src/input/reference_input_producer.cpp
  src/input/reset_target_input_producer.cpp
  src/input/state_input_producer.cpp
  src/output/cmd_vel_output_consumer.cpp
  src/output/nmpc_output_consumer.cpp
  src/state_machine/health_monitor_state.cpp
  src/state_machine/self_check_state.cpp
  src/state_machine/ready_state.cpp
  src/state_machine/reset_state.cpp
  src/state_machine/custom1_state.cpp)
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  unicycle_ugv_controller_nmpc_runtime
  xgc2_state_machine::state_machine
  xgc2_math::control
  xgc2_math::trajectory
  ${catkin_LIBRARIES}
  pthread)

install(TARGETS unicycle_ugv_controller_nmpc_runtime ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES ${UGV_NMPC_SOLVER_H}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/nmpc)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "results" EXCLUDE)

if(CATKIN_ENABLE_TESTING)
  # Diagnostic replay uses the production law; no second controller formula.
  add_executable(flatness_reference_replay tools/flatness_reference_replay.cpp)
  add_test(NAME flatness_skid_response
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/flatness_skid_response_test.py
      $<TARGET_FILE:flatness_reference_replay>)
  add_dependencies(flatness_reference_replay ${catkin_EXPORTED_TARGETS})
  target_link_libraries(flatness_reference_replay
    unicycle_ugv_controller_nmpc_runtime ${catkin_LIBRARIES})
  catkin_add_gtest(unicycle_ugv_controller_runtime_test
    test/unicycle_ugv_controller_runtime_test.cpp
    src/reference_cache.cpp
    src/unicycle_ugv_controller.cpp
    src/state_machine/health_monitor_state.cpp
    src/state_machine/self_check_state.cpp
    src/state_machine/ready_state.cpp
    src/state_machine/reset_state.cpp
    src/state_machine/custom1_state.cpp)
  if(TARGET unicycle_ugv_controller_runtime_test)
    target_link_libraries(unicycle_ugv_controller_runtime_test
      unicycle_ugv_controller_nmpc_runtime
      xgc2_state_machine::state_machine
      xgc2_math::control
      xgc2_math::trajectory
      ${catkin_LIBRARIES}
      gtest_main)
  endif()
  find_program(PYTHON3_EXECUTABLE python3)
  if(PYTHON3_EXECUTABLE)
    add_test(NAME unicycle_tracker_python
      COMMAND ${PYTHON3_EXECUTABLE} -m unittest discover -s
              ${CMAKE_CURRENT_SOURCE_DIR}/tools/unicycle_tracker/tests -p "test_*.py")
    set_tests_properties(unicycle_tracker_python PROPERTIES
      ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools")
  endif()
endif()
