cmake_minimum_required(VERSION 2.8.3)
project(detector)

find_package(
  catkin REQUIRED COMPONENTS
  mbzirc_husky REQUIRED
  cmake_modules REQUIRED
  roscpp REQUIRED
  std_msgs 
  sensor_msgs REQUIRED
  geometry_msgs REQUIRED
  pcl_ros REQUIRED
  tf2 REQUIRED
  message_generation REQUIRED
)

find_package(OpenCV REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_service_files(
        FILES
        brick_pile_trigger.srv
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs
)

catkin_package()

message(STATUS "addding detector")
add_executable(detector src/detector.cpp)

target_link_libraries(detector 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
