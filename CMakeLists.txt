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
    )
find_package(PCL REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

add_action_files(
    DIRECTORY action 
	FILES spray.action
)

generate_dynamic_reconfigure_options(
  	cfg/spray.cfg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)

add_executable(spray src/spray.cpp)

add_dependencies(spray ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries (spray  ${catkin_LIBRARIES})

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
