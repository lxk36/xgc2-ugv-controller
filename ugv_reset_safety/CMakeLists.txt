cmake_minimum_required(VERSION 3.0.2)
project(ugv_reset_safety)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs roscpp message_generation xgc2_geometry_msgs)
find_package(Eigen3 REQUIRED)
# Eigen's config exports uppercase variables, while catkin follows the spelling
# in DEPENDS when exporting public includes to CMake and pkg-config consumers.
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
add_message_files(FILES ResetRequest.msg ResetResponse.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)
catkin_package(INCLUDE_DIRS include LIBRARIES ugv_reset_safety_math CATKIN_DEPENDS geometry_msgs std_msgs roscpp message_runtime xgc2_geometry_msgs DEPENDS Eigen3)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_library(ugv_reset_safety_math src/reset_geometry.cpp src/reset_path.cpp src/reeds_shepp_distance.cpp)
target_link_libraries(ugv_reset_safety_math PUBLIC Eigen3::Eigen)
add_executable(ugv_reset_coordinator_node src/reset_coordinator_node.cpp)
add_dependencies(ugv_reset_coordinator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ugv_reset_coordinator_node ugv_reset_safety_math ${catkin_LIBRARIES})
install(TARGETS ugv_reset_safety_math ugv_reset_coordinator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md THIRD_PARTY_NOTICES DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/launch_control_with_slot_poses.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/reset_coordinator.test)
  add_rostest(test/reset_coordinator_scout.test)
  add_rostest(test/reset_cohort.test)
  add_test(NAME slot_initial_pose_launch_args_test
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/slot_initial_pose_launch_args_test.py)
  set_tests_properties(slot_initial_pose_launch_args_test PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME launch_control_with_slot_poses_rosrun_path
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/slot_pose_mapper_rosrun_path_test.py
            ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/launch_control_with_slot_poses.py
            ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/launch_control_with_slot_poses.py)
  foreach(case scene_projection reset_path reset_dwa reset_closed_loop reset_scenarios reset_session fleet_schedule)
    catkin_add_gtest(${case}_test test/${case}_test.cpp)
    if(TARGET ${case}_test)
      add_dependencies(${case}_test ${catkin_EXPORTED_TARGETS})
      target_link_libraries(${case}_test ugv_reset_safety_math ${catkin_LIBRARIES} gtest_main)
    endif()
  endforeach()
endif()
