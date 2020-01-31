cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_husky)
set(CMAKE_CXX_FLAGS "-ggdb -std=c++0x ${CMAKE_C_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS 
    actionlib
    actionlib_msgs
    geometry_msgs 
    message_generation
    move_base_msgs 
    roscpp 
    dynamic_reconfigure
    rospy 
    sensor_msgs
    std_msgs 
    image_transport
    tf
    mrs_lib
    mbzirc_husky_msgs
  )

find_package(PCL REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

add_action_files(
  DIRECTORY action 
	FILES spray.action
	FILES pickup.action
)

generate_dynamic_reconfigure_options(
  	cfg/spray.cfg
  	cfg/pickup.cfg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
    geometry_msgs
   )

catkin_package(
	CATKIN_DEPENDS message_runtime mbzirc_husky_msgs
  )

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} 
)

add_executable(brickDetector src/brickDetector.cpp src/CRawDepthImage.cpp src/CRawImage.cpp src/CSegmentation.cpp)
add_executable(pickup src/pickup.cpp src/CTimer.cpp)
add_executable(spray src/spray.cpp src/CPump.cpp src/CTimer.cpp)
add_dependencies(spray ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(pickup ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(brickDetector ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries (spray ${catkin_LIBRARIES})
target_link_libraries (pickup ${catkin_LIBRARIES})
target_link_libraries (brickDetector ${catkin_LIBRARIES})

#############
## Install ##
#############

 install(PROGRAMS
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# install(TARGETS mbzirc_husky
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
