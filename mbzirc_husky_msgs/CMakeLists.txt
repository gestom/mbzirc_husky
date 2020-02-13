cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_husky_msgs)

set(CMAKE_CXX_FLAGS "-std=c++17 -fno-diagnostics-color")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  std_srvs
  mrs_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  cmake_modules
)

add_message_files(DIRECTORY msg/ FILES
  Gen3ArmStatus.msg
  ArmStatus.msg
  brickPosition.msg
  brickGoal.msg
  brickCompleted.msg
)

add_service_files(DIRECTORY srv/ FILES
  Vector3.srv
  Vector7.srv
  Float64.srv
  EndEffectorPose.srv
  brickDetect.srv
  StoragePosition.srv
  LowerBrick.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  mrs_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime sensor_msgs mrs_msgs
  LIBRARIES mbzirc_husky_msgs
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(mbzirc_husky_msgs src/main.cpp)

add_dependencies(mbzirc_husky_msgs
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mbzirc_husky_msgs
  ${catkin_LIBRARIES}
)

