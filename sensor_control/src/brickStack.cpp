#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <mbzirc_husky_msgs/brickDetect.h>
#include <mbzirc_husky_msgs/brickPosition.h>
#include <mbzirc_husky_msgs/StoragePosition.h>
#include <mbzirc_husky_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
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

ros::Subscriber subscriberBrickPose;
ros::Subscriber subscriberScan;

int activeStorage = 5;

typedef actionlib::SimpleActionServer<mbzirc_husky::brickStackAction> Server;
Server *server;

typedef enum{
	NONE,
	ROBOT_ALIGN_X_PHI,
	ROBOT_MOVE_TURN_MOVE,
	ROBOT_ALIGN_PHI,
	ROBOT_MOVE_CAM,
	ROBOT_MOVE_ODO,
	BEHAVIOUR_NUMBER
}EBehaviour;

const char *behStr[] = { 
	"None",
	"aligning x and phi",
	"moving and turning to align y",
	"aligning to correct angle",
	"moving along area",
	"moving by odometry",
	"number"
};

const char *stateStr[] = { 
	"Idle",
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
	"FAIL"
};

typedef enum{
	IDLE = 0,
	TEST1,
	TEST2,
	ARMRESET,		//arm goes to dock position
	ARMTOSTORAGE, 		//
	ARMGRASP, 		//
	ARMPICKUP,		//	
	ARMTOPLACEMENT,		//arm goes to position above the brick compartment
	BRICKPLACE,		//brick is put into the storage and magnet released
	BRICKRELEASE,		//brick is put into the storage and magnet released
	MOVE_TO_NEXT_POS,	//
	TERMINALSTATE,	 	//marks terminal state
	FINAL,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL,
	STATE_NUMBER
}EState;

const char* toStr(EState state)
{
	if (state < STATE_NUMBER) return stateStr[state];
}

const char* toStr(EBehaviour beh)
{
	if (beh < BEHAVIOUR_NUMBER) return behStr[beh];
}

float movementDistances[] = {1.8,1.2,0.3,0.3,1.2,0.3,0.3};

EState state = IDLE;
EState nextState = IDLE;
EState recoveryState = IDLE;
EBehaviour behaviour = NONE;
EBehaviour nextBehaviour = NONE;
EBehaviour recoveryBehaviour = NONE;

ros::Publisher twistPub;

float moveDistance = 0.4;
tf::TransformListener *listener;
geometry_msgs::PoseStamped anchorPose;
geometry_msgs::PoseStamped robotPose;
geometry_msgs::Twist spd;
float anchorAngle = 0;
float wallAngleOffset = 0;

mbzirc_husky_msgs::StoragePosition getStoragePosition(int storage)
{
	int storePosition[] = {0,1,2,2,0,1,3};
	int storeLevel[] = {0,0,0,1,1,1,2};
	mbzirc_husky_msgs::StoragePosition srv;
	if (storage >= 0 && storage < 8){
		srv.request.position = storePosition[storage];	
		srv.request.layer = storeLevel[storage];	
	} 
	return srv;
}

void setSpeed(geometry_msgs::Twist speed)
{
	float maxX = 0.20;
	float maxZ = 0.15;
	if (speed.linear.x > +maxX) speed.linear.x = +maxX;
	if (speed.linear.x < -maxX) speed.linear.x = -maxX;
	if (speed.angular.z > +maxZ) speed.angular.z = +maxZ;
	if (speed.angular.z < -maxZ) speed.angular.z = -maxZ;
	twistPub.publish(speed);
}
int updateRobotPosition()
{
	int inc = 0;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped tf_pose;
	float az = 0;
	try {
		pose.header.frame_id = "base_link";
		pose.header.stamp = ros::Time::now();
		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		listener->waitForTransform("/base_link","map",pose.header.stamp,ros::Duration(0.2));
		listener->transformPose("/map",pose,robotPose);
		return 0;
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		return -1;
	}
}


int robotMoveOdo(const sensor_msgs::LaserScanConstPtr &msg)
{
	spd.linear.x = spd.angular.z = 0;
	float dx = anchorPose.pose.position.x-robotPose.pose.position.x;
	float dy = anchorPose.pose.position.y-robotPose.pose.position.y;
	float dist = sqrt(dx*dx+dy*dy);
	spd.linear.x = (fabs(moveDistance) - dist + 0.1);
	if (moveDistance < 0) spd.linear.x = -spd.linear.x;
	if (dist > fabs(moveDistance)) {
		spd.linear.x = spd.angular.z = 0;
		behaviour = nextBehaviour;
		printf("Movement done: %.3f %.3f\n",dist,moveDistance); 
		return 0;
	}
	setSpeed(spd);
	return 1;
}

int moveRobot(float distance,EBehaviour nextBeh = NONE)
{
	anchorPose = robotPose;
	moveDistance = distance;
	printf("Move command:  %.3f\n",distance); 
	nextBehaviour = nextBeh;
	behaviour = ROBOT_MOVE_ODO;
	return 0;
}


bool isTerminal(EState state)
{
	if(state < TERMINALSTATE) return false;
	return true;
}

int armToStorage()
{
	mbzirc_husky_msgs::StoragePosition srv = getStoragePosition(activeStorage);
	srv.request.num_of_waypoints = 1;
	ROS_INFO("PREPARING ARM TO %d, LAYER %d", srv.request.position , srv.request.layer);
	if (armStorageClient.call(srv)) {
		ROS_INFO("ARM READY FOR PICKUP");
		return 0;
	}
	return -1;
}

int graspBrick()
{
	mbzirc_husky_msgs::StoragePosition srv = getStoragePosition(activeStorage);
	srv.request.num_of_waypoints = 1;
	ROS_INFO("PICKUP FROM %d, LAYER %d", srv.request.position , srv.request.layer);
	if (graspBrickClient.call(srv)) {
		ROS_INFO("BRICK PICKED UP");
		return 0;
	}
	return -1;
}

int liftBrick()
{
	ROS_INFO("LIFTING BRICK");
	mbzirc_husky_msgs::StoragePosition srv = getStoragePosition(activeStorage);

	if (liftBrickClient.call(srv)){
		ROS_INFO("BRICK LIFTED");
		return 0;
	}
	ROS_INFO("BRICK LIFT FAILED");
	return -1;
}


int resetArm()
{
	ROS_INFO("RESETTING ARM INTO POSITION");
	std_srvs::Trigger srv;
	if (homeClient.call(srv)){
		ROS_INFO("ARM RESET");
		return 0;
	}
	ROS_INFO("ARM RESET FAILED");
	return -1;
}

int positionArm(bool high = true)
{
	ROS_INFO("MOVING ARM INTO WALL POSITION");
	mbzirc_husky_msgs::Float64 srv;
	srv.request.data = 0;
	if (prepareClient.call(srv)) {
		ROS_INFO("ARM POSITIONED");
		return 0;
	}
	ROS_INFO("ARM POSITION FAILED");
	return -1;
}

int placeBrick(float position = 0.25)
{
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

int releaseBrick()
{
	ROS_INFO("RESETTING ARM INTO POSITION");
	std_srvs::Trigger srv;
	if (releaseClient.call(srv)){
		ROS_INFO("ARM RESET");
		return 0;
	}
	ROS_INFO("ARM RESET FAILED");
	return -1;
}


void actionServerCallback(const mbzirc_husky::brickStackGoalConstPtr &goal, Server* as)
{
	mbzirc_husky::brickStackResult result;

	nextState = ARMTOSTORAGE;

	while (isTerminal(state) == false && ros::ok()) 
	{
		printf("Active behaviour %s, active state %s\n",toStr(behaviour),toStr(state));
		if (behaviour == NONE){
			state = nextState;
			switch (state){
				case TEST1: if (moveRobot(-2.5) == 0) nextState = TEST2; else nextState = IDLE; break; 
				case TEST2: if (moveRobot(+2.5) == 0) nextState = TEST1; else nextState = IDLE; break;
				case ARMRESET:  if (resetArm() == 0) nextState = ARMTOSTORAGE; else nextState = ARMRESET; break;
				case ARMTOSTORAGE:moveRobot(0.7); if (armToStorage() == 0) {nextState = ARMGRASP;} else nextState = ARMTOSTORAGE; break;
				case ARMGRASP: if (graspBrick() == 0) {nextState = ARMPICKUP;} else nextState = ARMTOSTORAGE; break;
				case ARMPICKUP: if (liftBrick() == 0) {nextState = ARMTOPLACEMENT;} else nextState = ARMTOSTORAGE; break;
				case ARMTOPLACEMENT: if (positionArm() == 0) {nextState = BRICKPLACE;} else nextState = ARMTOSTORAGE; break;
				case BRICKPLACE: if (placeBrick() == 0){nextState = BRICKRELEASE;} else {nextState = ARMRESET;} break;
				case BRICKRELEASE: if (releaseBrick() == 0){nextState = ARMTOSTORAGE;activeStorage--;} else {nextState = ARMTOSTORAGE;}
			}
		}
		usleep(1200000);
	}
	if (state == FINAL) state = SUCCESS; else state = FAIL;
	if (state == SUCCESS) 	server->setSucceeded(result);
	if (state == FAIL) 	server->setAborted(result);
	if (state == PREEMPTED) server->setPreempted(result);
	state = IDLE;	
}

void scanCallBack(const sensor_msgs::LaserScanConstPtr &msg) 
{
	if (updateRobotPosition() < 0) return;
	//if (behaviour == ROBOT_MOVE_SCAN)  robotMoveScan(msg); 
	//if (behaviour == ROBOT_MOVE_TURN_MOVE)  robotMTM(msg); 
	if (behaviour == ROBOT_MOVE_ODO)  robotMoveOdo(msg); 
	return;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "brickStack");
	ros::NodeHandle n;
	// Dynamic reconfiguration server
	/*dynamic_reconfigure::Server<mbzirc_husky::sprayConfig> dynServer;
  	dynamic_reconfigure::Server<mbzirc_husky::sprayConfig>::CallbackType f = boost::bind(&callback, _1, _2);
  	dynServer.setCallback(f);*/

	listener = new tf::TransformListener();
	subscriberScan = n.subscribe("/scan", 1, &scanCallBack);
	twistPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	prepareClient       = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/prepare_gripping");
	placeClient         = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/place_brick");
	homeClient          = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/home_arm");
	armStorageClient    = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/goto_storage");
	graspBrickClient    = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/pickup_brick_storage");
	liftBrickClient     = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/lift_brick_storage");
	releaseClient     = n.serviceClient<std_srvs::Trigger>("/husky/gripper/ungrip");

	server = new Server(n, "brickStackServer", boost::bind(&actionServerCallback, _1, server), false);
	server->start();

	while (ros::ok()){
		if (server->isPreemptRequested() && state != IDLE) state = PREEMPTED;
		if (state == STOPPING){
			state = IDLE;
		} 
		ros::spinOnce();
		usleep(1000);
	}
}
