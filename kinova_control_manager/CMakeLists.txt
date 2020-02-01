cmake_minimum_required(VERSION 2.8.3)
project(kinova_control_manager)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  geometry_msgs
  tf
  tf2
  tf2_geometry_msgs
  kinova_msgs
  kinova_driver
  kinova_bringup
  kinova_control
  kinova_description
  mrs_msgs
  mbzirc_husky_msgs
  # mrs_lib
  # moveit_ros_planning_interface
  # sensor_msgs
  # moveit_msgs
  # visualization_msgs
)


# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  LIBRARIES kinova_control_manager
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(kinova_control_manager src/velocity_control.cpp)

target_link_libraries(kinova_control_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(
  kinova_control_manager
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
