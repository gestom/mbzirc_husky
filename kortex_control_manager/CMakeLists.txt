cmake_minimum_required(VERSION 2.8.3)
project(kortex_control_manager)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  kortex_control
  kortex_driver
  mbzirc_husky_msgs
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  mrs_msgs
  pluginlib
  pouring_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)


# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface pouring_msgs
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(kortex_control_manager src/control_manager.cpp)
add_executable(fake_manager src/fake_manager.cpp)

target_link_libraries(kortex_control_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(fake_manager
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(
  kortex_control_manager
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

add_dependencies(
  fake_manager 
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
