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
    #    mrs_lib
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
	FILES fireExplore.action
	FILES brickExplore.action
	FILES brickPickup.action
	FILES brickStack.action
	)

generate_dynamic_reconfigure_options(
	cfg/spray.cfg
	cfg/pickup.cfg
	cfg/detectBrick.cfg
	cfg/brick_pile.cfg
	cfg/bounds.cfg
	cfg/symbolicMap.cfg
	)

add_service_files(
  FILES
  setPoi.srv
  getPoi.srv
  accessibleInventory.srv
  addInventory.srv
  getInventory.srv
  removeInventory.srv
  nextBrickPlacement.srv
  brickBuilt.srv
  debugFillInventory.srv
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
add_executable(fireExplore src/fireExplore.cpp)
add_executable(fireStateMachine src/fireStateMachine.cpp)
add_executable(brickStateMachine src/brickStateMachine.cpp)
add_executable(brickExplore src/brickExplore.cpp)
add_executable(brickPickup src/brickPickup.cpp)
add_executable(brickStack src/brickStack.cpp)
add_executable(symbolicMap src/symbolicMap.cpp src/order.cpp)
add_dependencies(spray ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(pickup ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(brickDetector ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(fireExplore ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(fireStateMachine ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(brickStateMachine ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(brickExplore ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(brickPickup ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS} kinova_control_manager_gencpp)
add_dependencies(brickStack ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(symbolicMap ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries (spray ${catkin_LIBRARIES})
target_link_libraries (pickup ${catkin_LIBRARIES})
target_link_libraries (brickDetector ${catkin_LIBRARIES})
target_link_libraries (fireStateMachine ${catkin_LIBRARIES})
target_link_libraries (fireExplore ${catkin_LIBRARIES})
target_link_libraries (brickStateMachine ${catkin_LIBRARIES})
target_link_libraries (brickExplore ${catkin_LIBRARIES})
target_link_libraries (brickPickup ${catkin_LIBRARIES})
target_link_libraries (brickStack ${catkin_LIBRARIES})
target_link_libraries (symbolicMap ${catkin_LIBRARIES})

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
