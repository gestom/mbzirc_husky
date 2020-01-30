cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_husky)

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
    tf
    mrs_lib
    kinova_msgs
    kinova_driver
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
add_message_files(DIRECTORY kinova_control_manager/msg FILES
	ArmStatus.msg
	)
add_service_files(DIRECTORY kinova_control_manager/srv FILES
	EndEffectorPose.srv
	Vector3.srv
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
	CATKIN_DEPENDS 	message_runtime
  )

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} 
)

add_executable(pickup src/pickup.cpp src/CTimer.cpp)
add_executable(spray src/spray.cpp src/CPump.cpp src/CTimer.cpp)
add_dependencies(spray ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(pickup ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries (spray ${catkin_LIBRARIES})
target_link_libraries (pickup  ${catkin_LIBRARIES})

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
