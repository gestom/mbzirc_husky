cmake_minimum_required(VERSION 2.8.3)
project(kinova_control_manager)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  nodelet
  kinova_msgs
  kinova_driver
  std_msgs
  geometry_msgs
  tf
  mrs_msgs
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

add_message_files(DIRECTORY msg FILES
  ArmStatus.msg
)

add_service_files(DIRECTORY srv FILES
  EndEffectorPose.srv
  Vector3.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  LIBRARIES kinova_control_manager
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(kinova_control_manager src/arm_manager.cpp)

target_link_libraries(kinova_control_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(
  kinova_control_manager
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
