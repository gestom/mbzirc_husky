#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <mbzirc_husky/nextBrickPlacement.h>
#include <geometry_msgs/Point.h>
#include <mbzirc_husky_msgs/brickDetect.h>
#include <mbzirc_husky_msgs/brickPosition.h>
#include <mbzirc_husky_msgs/StoragePosition.h>
#include <mbzirc_husky_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
#include <mbzirc_husky/brickBuilt.h>
#include <mbzirc_husky/removeInventory.h>
#include <mbzirc_husky/brickStackAction.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

ros::ServiceClient homeClient;
ros::ServiceClient armStorageClient;
ros::ServiceClient graspBrickClient;
ros::ServiceClient liftBrickClient;
ros::ServiceClient prepareClient;
ros::ServiceClient releaseClient;
ros::ServiceClient placeClient;
ros::ServiceClient disposeClient;
ros::ServiceClient inventoryQueryClient;
ros::ServiceClient inventoryBuiltClient;
ros::ServiceClient inventoryRemoveClient;

ros::Subscriber subscriberBrickPose;
ros::Subscriber subscriberScan;
ros::Subscriber subscriberPattern;
ros::Subscriber subscriberOdom;

boolean end_of_pattern = false;

/* int   activeStorage   = 5; */
float robotMoveDistance = 0;
float currentRobotPosition = 0;
int pattern_end_accumulator = 0;

typedef actionlib::SimpleActionServer<mbzirc_husky::brickStackAction> Server;
Server *                                                              server;

typedef enum
{
  NONE,
  ROBOT_ALIGN_X_PHI,
  ROBOT_MOVE_TURN_MOVE,
  ROBOT_ALIGN_PHI,
  ROBOT_MOVE_CAM,
  ROBOT_MOVE_ODO,
  BEHAVIOUR_NUMBER
} EBehaviour;

const char *behStr[] = {"None",  "aligning x and phi", "moving and turning to align y", "aligning to correct angle", "moving along area", "moving by odometry",
                        "number"};

const char *stateStr[] = {"Idle",
                          "TEST1",
                          "TEST2",
                          "resetting arm",
                          "positioning arm",
                          "aligning robot to a brick",
                          "aligning arm",
                          "descending arm",
                          "picking up",
                          "preparing to store",
                          "storing",
                          "move to red brick pile 2 for align",
                          "align with the wall",
                          "move to the green brick 1",
                          "move to the green brick 2",
                          "move to the blue brick",
                          "move to the red brick 2 for pickup",
                          "TERMINALSTATE",
                          "FINAL",
                          "STOPPING",
                          "PREEMPTED",
                          "SUCCESS",
                          "FAIL"};

typedef enum
{
  IDLE = 0,
  TEST1,
  TEST2,
  ARMRESET,          // arm goes to home position
  ARMTOSTORAGE,      // arm reaches to storage
  ARMGRASP,          // move down until magnet attaches
  ARMPICKUP,         // move up
  ARMTOPLACEMENT,    // arm goes to position above the placement pattern
  BRICKPLACE,        // magnet is moved to to a given height
  BRICKRELEASE,      // brick is put into the storage and magnet released
  MOVE_TO_NEXT_POS,  // drive to position to place next brick
  TERMINALSTATE,     // marks terminal state
  FINAL,
  STOPPING,
  PREEMPTED,
  SUCCESS,
  FAIL,
  STATE_NUMBER
} EState;

const char *toStr(EState state) {
  if (state < STATE_NUMBER)
    return stateStr[state];
}

const char *toStr(EBehaviour beh) {
  if (beh < BEHAVIOUR_NUMBER)
    return behStr[beh];
}

float movementDistances[] = {1.8, 1.2, 0.3, 0.3, 1.2, 0.3, 0.3};

EState     state             = IDLE;
EState     nextState         = IDLE;
EState     recoveryState     = IDLE;
EBehaviour behaviour         = NONE;
EBehaviour nextBehaviour     = NONE;
EBehaviour recoveryBehaviour = NONE;

ros::Publisher twistPub;

float                      moveDistance = 0.4;
tf::TransformListener *    listener;
geometry_msgs::PoseStamped anchorPose;
geometry_msgs::PoseStamped robotPose;
geometry_msgs::PoseStamped robotOdoPose;
geometry_msgs::Twist       spd;
float                      anchorAngle     = 0;
float                      wallAngleOffset = 0;

int next_brick_type;
int next_storage_position;
int next_storage_layer;
int next_wall_index;
int next_wall_layer;
int grasp_attempts = 0;

void odoCallBack(const nav_msgs::OdometryConstPtr &msg) 
{
	robotOdoPose.pose.position.x = msg->pose.pose.position.x;
	robotOdoPose.pose.position.y = msg->pose.pose.position.y;
	robotOdoPose.pose.position.z = 0;
	robotOdoPose.pose.orientation = msg->pose.pose.orientation;
	return;
}

mbzirc_husky_msgs::StoragePosition getStoragePosition(int storage) {
  int                                storePosition[] = {0, 1, 2, 2, 0, 1, 3};
  int                                storeLevel[]    = {0, 0, 0, 1, 1, 1, 2};
  mbzirc_husky_msgs::StoragePosition srv;
  if (storage >= 0 && storage < 8) {
    srv.request.position = storePosition[storage];
    srv.request.layer    = storeLevel[storage];
  }
  return srv;
}

void setSpeed(geometry_msgs::Twist speed) {
  float maxX = 0.20;
  float maxZ = 0.15;
  if (speed.linear.x > +maxX)
    speed.linear.x = +maxX;
  if (speed.linear.x < -maxX)
    speed.linear.x = -maxX;
  if (speed.angular.z > +maxZ)
    speed.angular.z = +maxZ;
  if (speed.angular.z < -maxZ)
    speed.angular.z = -maxZ;
  twistPub.publish(speed);
}

int updateRobotPosition() {
	robotPose = robotOdoPose;
	return 0;
}

int updateRobotPositiona() {
  int                        inc = 0;
  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseStamped tf_pose;
  float                      az = 0;
  try {
    pose.header.frame_id    = "base_link";
    pose.header.stamp       = ros::Time::now();
    pose.pose.position.x    = 0;
    pose.pose.position.y    = 0;
    pose.pose.position.z    = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    listener->waitForTransform("/base_link", "map", pose.header.stamp, ros::Duration(0.2));
    listener->transformPose("/map", pose, robotPose);
    return 0;
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return -1;
  }
}


int robotMoveOdo(const sensor_msgs::LaserScanConstPtr &msg) 
{
  spd.linear.x = spd.angular.z = 0;
  float dx                     = anchorPose.pose.position.x - robotPose.pose.position.x;
  float dy                     = anchorPose.pose.position.y - robotPose.pose.position.y;
  float dist                   = sqrt(dx * dx + dy * dy);
  spd.linear.x                 = (fabs(moveDistance) - dist + 0.1);
  if (moveDistance < 0) spd.linear.x = -spd.linear.x;
  if (dist > fabs(moveDistance)) {
	  spd.linear.x = spd.angular.z = 0;
	  behaviour                    = nextBehaviour;
	  if (moveDistance > 0)	  currentRobotPosition = currentRobotPosition + dist;
	  if (moveDistance < 0)	  currentRobotPosition = currentRobotPosition - dist;
	  printf("Movement done: %.3f %.3f\n", dist, moveDistance);
	  return 0;
  }
  setSpeed(spd);
  return 1;
}

int moveRobot(float distance, EBehaviour nextBeh = NONE) 
{
  while (updateRobotPosition() < 0){
  	usleep(50000);
	ros::spinOnce();
  }
  anchorPose   = robotPose;
  moveDistance = distance;
  printf("Move command:  %.3f\n", distance);
  nextBehaviour = NONE;
  behaviour     = ROBOT_MOVE_ODO;
  return 0;
}


bool isTerminal(EState state) {
  if (state < TERMINALSTATE)
    return false;
  return true;
}

int armToStorage() {
  mbzirc_husky_msgs::StoragePosition srv;
  srv.request.layer            = next_storage_layer;
  srv.request.position         = next_storage_position;
  srv.request.num_of_waypoints = 0;
  // mbzirc_husky_msgs::StoragePosition srv = getStoragePosition(activeStorage);
  // srv.request.num_of_waypoints           = 1;
  ROS_INFO("PREPARING ARM TO %d, LAYER %d", srv.request.position, srv.request.layer);
  if (armStorageClient.call(srv)) {
    ROS_INFO("ARM READY FOR PICKUP");
    return 0;
  }
  return -1;
}

int graspBrick() {
  mbzirc_husky_msgs::StoragePosition srv;
  srv.request.layer            = next_storage_layer;
  srv.request.position         = next_storage_position;
  srv.request.num_of_waypoints = 0;
  ROS_INFO("PICKUP FROM %d, LAYER %d", srv.request.position, srv.request.layer);
  if (graspBrickClient.call(srv)) {
    ROS_INFO("BRICK PICKED UP");

    return 0;
  }
  return -1;
}

int liftBrick() {
  ROS_INFO("LIFTING BRICK");
  mbzirc_husky_msgs::StoragePosition srv;
  srv.request.position         = next_storage_position;
  srv.request.layer            = next_storage_layer;
  srv.request.num_of_waypoints = 0;

  if (liftBrickClient.call(srv)) {
    ROS_INFO("BRICK LIFTED");
    return 0;
  }
  ROS_INFO("BRICK LIFT FAILED");
  return -1;
}


int resetArm() {
  ROS_INFO("RESETTING ARM INTO POSITION");
  std_srvs::Trigger srv;
  if (homeClient.call(srv)) {
    ROS_INFO("ARM RESET");
    return 0;
  }
  ROS_INFO("ARM RESET FAILED");
  return -1;
}

int positionArm(bool high = true) {
  ROS_INFO("MOVING ARM INTO WALL POSITION");
  std_srvs::Trigger srv;
  if (prepareClient.call(srv)) {
    ROS_INFO("ARM POSITIONED");
    return 0;
  }
  ROS_INFO("ARM POSITION FAILED");
  return -1;
}

int placeBrick(float position = 0.25) {
  ROS_INFO("MOVING ARM TO PLACE THE BRICK ");
  mbzirc_husky_msgs::Float64 srv;
  srv.request.data = position;
  if (placeClient.call(srv)) {
    ROS_INFO("ARM POSITIONED");
    return 0;
  }
  ROS_INFO("ARM POSITION FAILED");
  return -1;
}

int releaseBrick() {
  ROS_INFO("RELEASING BRICK");
  std_srvs::Trigger srv;
  if (releaseClient.call(srv)) {
    ROS_INFO("BRICK RELEASED");
    return 0;
  }
  ROS_INFO("ARM RESET FAILED");
  return -1;
}

int disposeBrick() {
  ROS_INFO("DISPOSING BRICK P%d, L%d", next_storage_position, next_storage_layer);
  mbzirc_husky_msgs::StoragePosition srv;
  srv.request.position = next_storage_position;
  srv.request.layer    = next_storage_layer;
  if (disposeClient.call(srv)) {
    ROS_INFO("BRICK DISPOSED");
    return 0;
  }
  ROS_INFO("BRICK DISPOSAL FAILED");
  return -1;
}

// TODO
int moveToNextPosition(){
  
}

int fetchNextBrickData() {
  // query inventory for next brick
  mbzirc_husky::nextBrickPlacement srv;
  srv.request.offset = 0.07;
  if (inventoryQueryClient.call(srv.request, srv.response)) {
    if (srv.response.brickType == -1) {
      ROS_INFO("[%s]: No next brick", ros::this_node::getName().c_str());
      return -1;
    }
    robotMoveDistance 	  = srv.response.wallOriginOffset - currentRobotPosition; 
    printf("NEXT MOVE: Position: %f Inventory: %f Command: %f\n",currentRobotPosition,srv.response.wallOriginOffset,robotMoveDistance);
    next_brick_type       = srv.response.brickType;
    next_storage_position = srv.response.position;
    next_storage_layer    = srv.response.layer;
    next_wall_index       = srv.response.wallIndex;
    next_wall_layer       = srv.response.wallLayer;
    ROS_INFO("[%s]: Next brick data loaded: T:%d, P:%d, L:%d, WI: %d, WL: %d", ros::this_node::getName().c_str(), next_brick_type, next_storage_position,
             next_storage_layer, next_wall_layer, next_wall_index);
    return 0;
  }
  return -1;
}

int eraseFromInventory() {
  ROS_WARN("BRICK ON POS %d, LAYER %d REMOVED FROM INVENTORY", next_storage_position, next_storage_layer);
  mbzirc_husky::removeInventory srv;
  srv.request.layer    = next_storage_layer;
  srv.request.position = next_storage_position;
  inventoryRemoveClient.call(srv);
  return 0;
}

int placedBrickInventory() {
  mbzirc_husky::brickBuilt inventSrv;
  inventSrv.request.fromLayer    = next_storage_layer;
  inventSrv.request.fromPosition = next_storage_position;
  inventSrv.request.wallLayer = next_wall_layer;
  inventSrv.request.wallIndex = next_wall_index;
  
  if (inventoryBuiltClient.call(inventSrv)) {
    ROS_INFO("[%s]: BRICK FROM POS %d AND LAYER %d WAS PLACED", ros::this_node::getName().c_str(), inventSrv.request.fromPosition, inventSrv.request.fromLayer);
    return 0;
  } else {
    ROS_INFO("INVENTORY UPDATE FAILED");
  }
  return -1;
}


void actionServerCallback(const mbzirc_husky::brickStackGoalConstPtr &goal, Server *as) {
  mbzirc_husky::brickStackResult result;

  nextState = ARMTOSTORAGE;
  if (fetchNextBrickData() == -1) {
    ROS_INFO("[%s]: INVENTORY EMPTY, ABORTING BRICK STACK", ros::this_node::getName().c_str());
    nextState = SUCCESS;
  }

  while (isTerminal(state) == false && ros::ok()) {
    printf("Active behaviour %s, active state %s\n", toStr(behaviour), toStr(state));
    if (behaviour == NONE || state == ARMTOSTORAGE || state == ARMRESET || state == ARMGRASP || state == ARMPICKUP || state == ARMTOPLACEMENT) {
      state = nextState;
      switch (state) {
        case TEST1:
          if (moveRobot(-2.5) == 0)
            nextState = TEST2;
          else
            nextState = IDLE;
          break;
        case TEST2:
          if (moveRobot(+2.5) == 0)
            nextState = TEST1;
          else
            nextState = IDLE;
          break;
        case ARMRESET:
          if (resetArm() == 0)
            nextState = ARMTOSTORAGE;
          else
            nextState = ARMRESET;
          break;
        case ARMTOSTORAGE:
	  if (grasp_attempts < 2) moveRobot(robotMoveDistance);
          if (armToStorage() == 0)
            nextState = ARMGRASP;
          else
            nextState = ARMTOSTORAGE;
          break;
        case ARMGRASP:
          ++grasp_attempts;
          if (graspBrick() == 0) {
            grasp_attempts = 0;
            nextState      = ARMPICKUP;
          } else {
            nextState = ARMTOSTORAGE;
          }
          if (grasp_attempts > 3) {
            grasp_attempts = 0;
            ROS_WARN("BRICK GRASP FAILED 3 TIMES IN A ROW");
            if (disposeBrick() == 0) {
              eraseFromInventory();
              if (fetchNextBrickData() == -1) {
                ROS_INFO("[%s]: INVENTORY EMPTY, ABORTING BRICK STACK", ros::this_node::getName().c_str());
                nextState = SUCCESS;
                break;
              } else {
                nextState = ARMTOSTORAGE;
              }
            }else{
              ROS_FATAL("BRICK DISPOSAL UNSUCCESSFUL!");
              // TODO ignore bricks in the current position
              nextState = FAIL;
            }
          }
          break;
        case ARMPICKUP:
          if (liftBrick() == 0) {
            nextState = ARMTOPLACEMENT;
          } else
            nextState = ARMTOSTORAGE;
          break;
        case ARMTOPLACEMENT:
          if (positionArm() == 0) {
            nextState = BRICKPLACE;
          } else {
            // disposeBrick(); // this is probably not necessary
            eraseFromInventory();
            if (fetchNextBrickData() == -1) {
              ROS_INFO("[%s]: INVENTORY EMPTY, ABORTING BRICK STACK", ros::this_node::getName().c_str());
              nextState = SUCCESS;
              break;
            }
            nextState = ARMTOSTORAGE;
          }
          break;
        case BRICKPLACE:
          if (placeBrick() == 0) {
            nextState = BRICKRELEASE;
          } else {
            nextState = ARMRESET;
            eraseFromInventory();
          }
          break;
        case BRICKRELEASE:
          if (releaseBrick() == 0) {
            nextState = ARMTOSTORAGE;
            placedBrickInventory();
          } else {
            nextState = ARMTOSTORAGE;
            eraseFromInventory();
          }
          if (fetchNextBrickData() == -1) {
            ROS_INFO("[%s]: INVENTORY EMPTY, ABORTING BRICK STACK", ros::this_node::getName().c_str());
            nextState = SUCCESS;
            break;
          }
      }
    }
    usleep(1200000);
  }
  if (state == FINAL)
    state = SUCCESS;
  else
    state = FAIL;
  if (state == SUCCESS)
    server->setSucceeded(result);
  if (state == FAIL)
    server->setAborted(result);
  if (state == PREEMPTED)
    server->setPreempted(result);
  state = IDLE;
}

void wallCallBack(const geometry_msgs::PointConstPtr &msg) {
	float angle = atan2(msg->y,msg->x);
	printf("Angle %f\n",angle);
	geometry_msgs::Twist spd;
	spd.angular.z = -angle;
	if (not end_of_pattern) {
        spd.linear.x = -0.1;
    }
	if (std::abs(msg->z) < 999){
	    pattern_end_accumulator = 0;
	    spd.angular.z = msg->z;
	}
	if (std::abs(msg->x) > 999){
	    pattern_end_accumulator++;
	    spd.linear.x = 0.0;
	}
	if (pattern_end_accumulator > 20){
	    /// TODO stop subscribing and start brick stacking
	    ROS_INFO("END OF THE PATTERN - START BRICK STACK");
	    end_of_pattern = true;
	}
	setSpeed(spd);
	return;
}


void scanCallBack(const sensor_msgs::LaserScanConstPtr &msg) {
  if (updateRobotPosition() < 0)
    return;
  // if (behaviour == ROBOT_MOVE_SCAN)  robotMoveScan(msg);
  // if (behaviour == ROBOT_MOVE_TURN_MOVE)  robotMTM(msg);
  if (behaviour == ROBOT_MOVE_ODO)
    robotMoveOdo(msg);
  return;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "brickStack");
  ros::NodeHandle n;
  // Dynamic reconfiguration server
  // dynamic_reconfigure::Server<mbzirc_husky::sprayConfig> dynServer;
  // dynamic_reconfigure::Server<mbzirc_husky::sprayConfig>::CallbackType f = boost::bind(&callback, _1, _2);
  // dynServer.setCallback(f);

  listener         = new tf::TransformListener();
  subscriberScan   = n.subscribe("/scan", 1, &scanCallBack);
  subscriberPattern   = n.subscribe("/wall_pattern_line", 1, &wallCallBack);
  subscriberOdom = n.subscribe("/odometry/filtered", 1, &odoCallBack);
  twistPub         = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  prepareClient    = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/prepare_placing");
  placeClient      = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/place_brick");
  homeClient       = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/home_arm");
  armStorageClient = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/goto_storage");
  graspBrickClient = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/pickup_brick_storage");
  liftBrickClient  = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/lift_brick_storage");
  releaseClient    = n.serviceClient<std_srvs::Trigger>("/husky/gripper/ungrip");
  disposeClient    = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/dispose_brick");

  inventoryQueryClient  = n.serviceClient<mbzirc_husky::nextBrickPlacement>("/inventory/nextBrickPlacement");
  inventoryBuiltClient  = n.serviceClient<mbzirc_husky::brickBuilt>("/inventory/brickBuilt");
  inventoryRemoveClient = n.serviceClient<mbzirc_husky::removeInventory>("/inventory/remove");

  server = new Server(n, "brickStackServer", boost::bind(&actionServerCallback, _1, server), false);
  server->start();

  while (ros::ok()) {
    if (server->isPreemptRequested() && state != IDLE)
      state = PREEMPTED;
    if (state == STOPPING) {
      state = IDLE;
    }
    ros::spinOnce();
    usleep(1000);
  }
}
