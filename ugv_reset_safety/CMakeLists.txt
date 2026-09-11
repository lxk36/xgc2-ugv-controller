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
find_package(xgc2_acados REQUIRED CONFIG)
find_path(RESET_OSQP_ROOT osqp/osqp.h HINTS /opt/xgc2/acados/include ${XGC2_ACADOS_INCLUDE_DIRS})
find_library(RESET_OSQP_LIBRARY osqp HINTS /opt/xgc2/acados/lib ${XGC2_ACADOS_LIBRARY_DIRS})
if(NOT RESET_OSQP_ROOT OR NOT RESET_OSQP_LIBRARY)
  message(FATAL_ERROR "xgc2-acados OSQP headers and library are required")
endif()
add_message_files(FILES ResetRequest.msg ResetResponse.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)
catkin_package(INCLUDE_DIRS include LIBRARIES ugv_reset_safety_math CATKIN_DEPENDS geometry_msgs std_msgs roscpp message_runtime xgc2_geometry_msgs DEPENDS Eigen3)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_library(ugv_reset_safety_math src/safety_filter.cpp src/reset_guidance.cpp)
target_include_directories(ugv_reset_safety_math PRIVATE ${RESET_OSQP_ROOT})
target_link_libraries(ugv_reset_safety_math PUBLIC Eigen3::Eigen PRIVATE ${RESET_OSQP_LIBRARY})
# Preserve the packaged solver dependency after CMake removes its build RPATH.
# A same-named OSQP from another installation must not win the default lookup.
get_filename_component(RESET_OSQP_LIBRARY_DIR "${RESET_OSQP_LIBRARY}" DIRECTORY)
set_target_properties(ugv_reset_safety_math PROPERTIES INSTALL_RPATH "${RESET_OSQP_LIBRARY_DIR}")
add_executable(ugv_reset_coordinator_node src/reset_coordinator_node.cpp)
add_dependencies(ugv_reset_coordinator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ugv_reset_coordinator_node ugv_reset_safety_math ${catkin_LIBRARIES})
install(TARGETS ugv_reset_safety_math ugv_reset_coordinator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/reset_coordinator.test)
  add_rostest(test/reset_coordinator_scout.test)
  foreach(case scene_projection safety_filter reset_guidance reset_closed_loop reset_scenarios reset_session fleet_schedule)
    catkin_add_gtest(${case}_test test/${case}_test.cpp)
    if(TARGET ${case}_test)
      add_dependencies(${case}_test ${catkin_EXPORTED_TARGETS})
      target_link_libraries(${case}_test ugv_reset_safety_math ${catkin_LIBRARIES} gtest_main)
    endif()
  endforeach()

  # Large statistical campaigns reuse the exact deterministic scenario plant and
  # independent rectangle collision audit, but are intentionally not attached to
  # the normal run_tests target. tools/run_reset_monte_carlo.py shards this binary
  # over many seeds; CI executes a small smoke campaign explicitly.
  add_executable(reset_monte_carlo test/reset_monte_carlo_test.cpp)
  add_dependencies(reset_monte_carlo ${catkin_EXPORTED_TARGETS})
  target_link_libraries(reset_monte_carlo ugv_reset_safety_math ${catkin_LIBRARIES} gtest_main)
endif()
