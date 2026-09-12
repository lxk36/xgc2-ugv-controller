cmake_minimum_required(VERSION 3.0.2)
project(mecanum_ugv_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp std_msgs ugv_reset_safety)
find_package(xgc2_state_machine REQUIRED CONFIG)
find_package(xgc2_math REQUIRED CONFIG)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs ugv_reset_safety
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
  src/common_types.cpp
  src/mecanum_ugv_controller.cpp
  src/mecanum_ugv_ros_node.cpp
  src/state_machine/states.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  xgc2_state_machine::state_machine
  xgc2_math::algebra
  ${catkin_LIBRARIES}
  pthread)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY tools
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(mecanum_ugv_controller_runtime_test
    test/mecanum_ugv_controller_runtime_test.cpp
    src/common_types.cpp
    src/mecanum_ugv_controller.cpp
    src/state_machine/states.cpp)
  if(TARGET mecanum_ugv_controller_runtime_test)
    target_link_libraries(mecanum_ugv_controller_runtime_test
      xgc2_state_machine::state_machine
      xgc2_math::algebra
      ${catkin_LIBRARIES}
      gtest_main)
  endif()
  find_package(rostest QUIET)
  if(rostest_FOUND)
    add_rostest_gtest(mecanum_ugv_ros_sandbox_test
      test/mecanum_ugv_ros_sandbox.test
      test/mecanum_ugv_ros_sandbox_test.cpp)
    if(TARGET mecanum_ugv_ros_sandbox_test)
      target_link_libraries(mecanum_ugv_ros_sandbox_test ${catkin_LIBRARIES})
      add_dependencies(mecanum_ugv_ros_sandbox_test ${PROJECT_NAME}_node)
    endif()
    add_rostest_gtest(mecanum_ugv_public_command_reset_test
      test/mecanum_ugv_public_command_reset.test
      test/mecanum_ugv_public_command_reset_test.cpp)
    if(TARGET mecanum_ugv_public_command_reset_test)
      target_link_libraries(mecanum_ugv_public_command_reset_test ${catkin_LIBRARIES})
      add_dependencies(mecanum_ugv_public_command_reset_test ${PROJECT_NAME}_node)
    endif()
  endif()
  find_program(PYTHON3_EXECUTABLE python3)
  if(PYTHON3_EXECUTABLE)
    add_test(NAME holonomic_tracker_python
      COMMAND ${PYTHON3_EXECUTABLE} -m unittest discover -s
              ${CMAKE_CURRENT_SOURCE_DIR}/tools/holonomic_tracker/tests -p "test_*.py")
    set_tests_properties(holonomic_tracker_python PROPERTIES
      ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools")
  endif()
endif()
