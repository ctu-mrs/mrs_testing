cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_testing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  mrs_msgs
  mrs_lib
  nav_msgs
  std_msgs
  std_srvs
  dynamic_reconfigure
  rostest
  )

generate_dynamic_reconfigure_options(
  cfg/circle_flier.cfg
  )

set(LIBRARIES
  RandomFlier TrajectoryRandomFlier PathRandomFlier CircleFlier TfTest EmulateSensorErrors
  )

set(EXECUTABLES
  ControlTest
  )

catkin_package(
  CATKIN_DEPENDS roscpp nodelet nav_msgs mrs_lib std_msgs std_srvs mrs_msgs dynamic_reconfigure rostest
  LIBRARIES ${LIBRARIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

add_executable(ControlTest
  test/control_test.cpp
  )

add_dependencies(ControlTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(ControlTest
  ${catkin_LIBRARIES}
  )

add_library(RandomFlier
  src/random_flier.cpp
  )

add_dependencies(RandomFlier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(RandomFlier
  ${catkin_LIBRARIES}
  )

add_library(TrajectoryRandomFlier
  src/trajectory_random_flier.cpp
  )

add_dependencies(TrajectoryRandomFlier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(TrajectoryRandomFlier
  ${catkin_LIBRARIES}
  )

add_library(PathRandomFlier
  src/path_random_flier.cpp
  )

add_dependencies(PathRandomFlier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(PathRandomFlier
  ${catkin_LIBRARIES}
  )

add_library(CircleFlier
  src/circle_flier.cpp
  )

add_dependencies(CircleFlier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(CircleFlier
  ${catkin_LIBRARIES}
  )

add_library(TfTest
  src/tf_test.cpp
  )

add_dependencies(TfTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(TfTest
  ${catkin_LIBRARIES}
  )

add_library(EmulateSensorErrors
  src/emulate_sensor_errors.cpp
  )

add_dependencies(EmulateSensorErrors
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(EmulateSensorErrors
  ${catkin_LIBRARIES}
  )
## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

add_rostest_gtest(
  ControlTestRostest
  test/control_test_rostest.launch
  test/control_test.cpp
  )

target_compile_definitions(ControlTestRostest PRIVATE ROSTEST=1)

target_link_libraries(
  ControlTestRostest
  ${catkin_LIBRARIES}
  )

add_dependencies(
  ControlTestRostest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch config worlds layouts rviz test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY tmux scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
