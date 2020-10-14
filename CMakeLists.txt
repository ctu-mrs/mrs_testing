cmake_minimum_required(VERSION 2.8.3)
project(mrs_uav_testing)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  mrs_msgs
  mrs_lib
  nav_msgs
  std_msgs
  std_srvs
  mavros_msgs
  dynamic_reconfigure
  rostest
  )

generate_dynamic_reconfigure_options(
  cfg/circle_flier.cfg
  )

catkin_package(
  CATKIN_DEPENDS nav_msgs mrs_lib std_msgs std_srvs mavros_msgs
  LIBRARIES RandomFlier TrajectoryRandomFlier CircleFlier
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  )

add_executable(ControlTest test/control_test.cpp)

add_dependencies(ControlTest
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(ControlTest
  ${catkin_LIBRARIES}
  )

add_library(RandomFlier src/random_flier.cpp)

add_dependencies(RandomFlier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(RandomFlier
  ${catkin_LIBRARIES}
  )

add_library(TrajectoryRandomFlier src/trajectory_random_flier.cpp)

add_dependencies(TrajectoryRandomFlier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(TrajectoryRandomFlier
  ${catkin_LIBRARIES}
  )

add_library(CircleFlier src/circle_flier.cpp)

add_dependencies(CircleFlier
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(CircleFlier
  ${catkin_LIBRARIES}
  )

#############
## Testing ##
#############

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
