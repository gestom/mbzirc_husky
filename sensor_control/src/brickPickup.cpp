#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <mbzirc_husky/brickPickupAction.h>
#include <mbzirc_husky_msgs/brickDetect.h>
#include <mbzirc_husky_msgs/brickPosition.h>
#include <mbzirc_husky_msgs/StoragePosition.h>
#include <mbzirc_husky_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

typedef actionlib::SimpleActionServer<mbzirc_husky::brickPickupAction> Server;
Server *server;

float moveDistance = 0.4;
typedef enum{
	NONE,
	ROBOT_ALIGN_X_PHI,
	ROBOT_MOVE_TURN_MOVE,
	ROBOT_ALIGN_PHI,
	ROBOT_ALIGN_Y,
	ROBOT_ALIGN_X,
	ROBOT_MOVE_ODO,
	BEHAVIOUR_NUMBER
}EBehaviour;

const char *behStr[] = { 
	"None",
	"aligning x and phi",
	"aligning phi",
	"move straight",
	"number"
};

EBehaviour behaviour = NONE;
EBehaviour nextBehaviour = NONE;
EBehaviour recoveryBehaviour = NONE;

int behaviourResult = 0;

const char *stateStr[] = { 
	"Idle",
	"TEST1",
	"TEST2",
	"ARMRESET",		
	"ARMPOSITIONING", 	
	"aligning robot to a brick",	
	"ARMPOSITIONING to the second brick", 	
	"ARMALIGNMENT",	
	"ARMDESCENT",
	"ARMPICKUP",
	"ARMSTORAGE",
	"BRICKSTORE",
	"move to the next brick",
	"align with the wall",
	"move to the green brick",
	"ARMLOWPOSITIONING",
	"ARMLOWALIGNMENT",
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
	ARMPOSITIONING, 	//arm goes to overview positions
	ARMPOSITIONING_NOMOVE, 	//arm goes to overview positions
	ROBOT_ALIGNMENT, 	//arm goes to overview positions
	ARMALIGNMENT,		 //arm makes fine alignment
	ARMDESCENT,		 //arm does down and detects the magnet feedback
	ARMPICKUP,		 //grasp
	ARMSTORAGE,		 //arm goes to position above the brick compartment
	BRICKSTORE,		 //brick is put into the storage and magnet released
	ROBOT_MOVE_NEXT_BRICK,	 //
	ROBOT_ALIGN_WITH_WALL,	 //
	MOVE_TO_GREEN_BRICK_1,	 //
	MOVE_TO_GREEN_BRICK_2,	 //
	ARMLOWPOSITIONING,	//usused atm
	ARMLOWALIGNMENT,	//unused atm
	TERMINALSTATE,	 //marks terminal state
	FINAL,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL,
	STATE_NUMBER
}EState;
EState state = IDLE;

const char* toStr(EState state)
{
	if (state < STATE_NUMBER) return stateStr[state];
}

const char* toStr(EBehaviour beh)
{
	if (beh < BEHAVIOUR_NUMBER) return behStr[beh];
}




ros::NodeHandle* pn;

ros::Publisher twistPub;

tf::TransformListener *listener;
geometry_msgs::PoseStamped anchorPose;
geometry_msgs::PoseStamped robotPose;
geometry_msgs::Twist spd;
float anchorAngle = 0;
float wallAngleOffset = 0;

//service clients for the arm
ros::ServiceClient service_client_brick_detector;
ros::ServiceClient prepareClient;
ros::ServiceClient liftClient;
ros::ServiceClient alignClient;
ros::ServiceClient pickupClient;
ros::ServiceClient homeClient;
ros::ServiceClient armStorageClient;
ros::ServiceClient brickStoreClient;
ros::ServiceClient brickDetectorClient;
ros::Subscriber subscriberBrickPose;
ros::Subscriber subscriberScan;

int active_storage = 0; // TODO make this an enum??;
int active_layer = 0; // TODO make this an enum??;
int incomingMessageCount = 0;

bool isTerminal(EState state)
{
	if (state < TERMINALSTATE) return false;
	if(state == FINAL) return true;
	return true;
}

float stateMove = -1;
int alignmentOK = 0;


void setSpeed(geometry_msgs::Twist speed)
{
	float maxX = 0.10;
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
	printf("Moving fw: %.3f %.3f\n",dist,moveDistance); 
	spd.linear.x = (fabs(moveDistance) - dist + 0.1);
	if (moveDistance < 0) spd.linear.x = -spd.linear.x;
	if (dist > moveDistance) {
		spd.linear.x = spd.angular.z = 0;
		behaviour = nextBehaviour;
		printf("Movement done: %.3f %.3f\n",dist,moveDistance); 
		return 0;
	}
	setSpeed(spd);
	return 1;
}

int robotMTM(const sensor_msgs::LaserScanConstPtr &msg)
{
	spd.linear.x = spd.angular.z = 0;
	float dx = anchorPose.pose.position.x-robotPose.pose.position.x;
	float dy = anchorPose.pose.position.y-robotPose.pose.position.y;
	float dist = sqrt(dx*dx+dy*dy);
	spd.linear.x = (fabs(moveDistance) - dist + 0.1);
	if (moveDistance < 0) spd.linear.x = - spd.linear.x;
	printf("MTM moving: %.3f %.3f\n",dist,moveDistance); 
	if (dist > fabs(moveDistance) && moveDistance < 0){
		behaviour = nextBehaviour; 	
	}
	if (dist > moveDistance && moveDistance > 0)
	{
		spd.linear.x = 0;
		float angleDiff = anchorAngle-tf::getYaw(robotPose.pose.orientation);
		printf("MTM turning: %.3f \n",angleDiff); 
		spd.angular.z =  angleDiff*10;
		if (fabs(angleDiff) < 0.01){
			anchorPose = robotPose;
			moveDistance = - moveDistance;
		}
	}
	setSpeed(spd);
	return 0;
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

int moveTurnMove(float distance,EBehaviour nextBeh = NONE)
{
	moveDistance = distance;
	nextBehaviour = nextBeh;
	behaviour = ROBOT_MOVE_TURN_MOVE;
	return 0;
}


int robotAlignXPhi(const mbzirc_husky_msgs::brickPositionConstPtr &msg)
{
	float angle = tf::getYaw(msg->pose.pose.orientation);
	geometry_msgs::Twist spd;
	printf("Robot align: %i %f %f %f\n", msg->detected, msg->pose.pose.position.x, msg->pose.pose.position.y, angle);
	spd.linear.x = spd.angular.z = 0;
	if (msg->detected){
		if (incomingMessageCount++ > 90) {
			float angleDiff = (msg->pose.pose.position.y*3-angle);
			spd.angular.z =  angleDiff*10;
			spd.linear.x = -msg->pose.pose.position.x;

			
			if (fabs(msg->pose.pose.position.y) < 0.02 && fabs(msg->pose.pose.position.x) < 0.02){
				spd.linear.x = spd.angular.z = 0;
				anchorAngle = tf::getYaw(anchorPose.pose.orientation)-angle;
				behaviour = NONE;
				return 0;
			}
			if ((fabs(angleDiff) < 0.01 && fabs(msg->pose.pose.position.x) < 0.02) || (angle*msg->pose.pose.position.y > 0 && fabs(angle) > 0.2)){
				anchorPose = robotPose;
				anchorAngle = tf::getYaw(anchorPose.pose.orientation)-angle;
				moveTurnMove(0.3,ROBOT_ALIGN_X_PHI);
				return 1;
			}
		}
	}
	setSpeed(spd);
}

void scanCallBack(const sensor_msgs::LaserScanConstPtr &msg) 
{
	if (updateRobotPosition() < 0) return;
	if (behaviour == ROBOT_MOVE_ODO)  robotMoveOdo(msg); 
	if (behaviour == ROBOT_MOVE_TURN_MOVE)  robotMTM(msg); 
	return;
}




int robotAlignPhi(const mbzirc_husky_msgs::brickPositionConstPtr &msg)
{
	float angle = tf::getYaw(msg->pose.pose.orientation);
	printf("Robot align to the wall: %f\n", angle);
	spd.linear.x = spd.angular.z = 0;
	if (msg->detected){
		if (incomingMessageCount++ > 90) {
			float angleDiff = wallAngleOffset-angle;
			spd.angular.z =  angleDiff*10;
			spd.linear.x = 0;
			if (fabs(angleDiff) < 0.01){
				behaviour = nextBehaviour;
				return 0;
			}
		}
	}
	setSpeed(spd);
}

int alignRobotWithWall(float offset = 0,EBehaviour nb=NONE)
{
	wallAngleOffset = offset;
	incomingMessageCount = -120;
	behaviour = ROBOT_ALIGN_PHI;
	nextBehaviour = nb;
}


void callbackBrickPose(const mbzirc_husky_msgs::brickPositionConstPtr &msg) 
{
	int inc = 0;
	float maxX = 0.10;
	float maxZ = 0.15;
	float angle = tf::getYaw(msg->pose.pose.orientation);
	if (updateRobotPosition() < 0) return;
	if (behaviour == ROBOT_ALIGN_X_PHI) behaviourResult = robotAlignXPhi(msg);
	if (behaviour == ROBOT_ALIGN_PHI) behaviourResult = robotAlignPhi(msg);
	return;
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
	ROS_INFO("MOVING ARM INTO POSITION");
	mbzirc_husky_msgs::Float64 srv;
	srv.request.data = 0;
	if (active_storage == 1) srv.request.data = -0.3;
	if (prepareClient.call(srv)) {
		usleep(3500000);	//TODO this is unsafe
		if (active_storage == 1) usleep(1700000); 
		ROS_INFO("ARM POSITIONED");
		return 0;
	}
	ROS_INFO("ARM POSITION FAILED");
	return -1;
}


int alignArm(bool high = true)
{
	ROS_INFO("ALIGNING ARM");
	mbzirc_husky_msgs::Float64 srv;
	srv.request.data = 0.0;
	if (active_storage == 1) srv.request.data = -0.3;
	if (alignClient.call(srv)) {
		usleep(3000000);
		ROS_INFO("ARM ALIGNED");
		return 0;
	}
	return -1;
}

int switchDetection(bool on)
{
	incomingMessageCount = 0; 
	mbzirc_husky_msgs::brickDetect brick_srv;
	brick_srv.request.activate            = on;
	brick_srv.request.groundPlaneDistance = 0;
	brick_srv.request.x                   = 640;
	brick_srv.request.y                   = 480;
	brickDetectorClient.call(brick_srv.request, brick_srv.response);
	return 0;
}

int descentArm()
{

	ROS_INFO("ARM DESCENDING");
	std_srvs::Trigger srv;
	if (pickupClient.call(srv)) {
		ROS_INFO("ARM DESCENDED");
		return 0;
	}
	ROS_INFO("FAILED TO DESCEND ARM, RE-ALIGNING");
	return -1;
}

int pickupBrick()
{
	ROS_INFO("RAISING ARM");
	mbzirc_husky_msgs::Float64 srv;
	srv.request.data = 0;
	if (liftClient.call(srv)) {
		ROS_INFO("BRICK PICK UP DONE");
		return 0;
	} 
	ROS_INFO("BRICK PICKUP FAILED");
	return -1;
}


int prepareStorage()
{
	mbzirc_husky_msgs::StoragePosition srv;
	srv.request.position = active_storage%3;
	srv.request.layer    = active_storage/3;
	ROS_INFO("STORING BRICK IN POSITION %d, LAYER %d", srv.request.position , srv.request.layer);
	if (armStorageClient.call(srv)) {
		ROS_INFO("BRICK READY FOR STORAGE");
		return 0;
	}
	return -1;
}


int storeBrick()
{
	mbzirc_husky_msgs::StoragePosition srv;
	srv.request.position = active_storage%3;
	srv.request.layer    = active_storage/3;
	ROS_INFO("STORING BRICK IN POSITION %d, LAYER %d", srv.request.position , srv.request.layer);
	if (brickStoreClient.call(srv)) {
		ROS_INFO("BRICK STORED IN POSITION %d", active_storage);
		active_storage++;
		return 0;
		
		/*if (active_storage == 2) {
			ROS_INFO("FINISHED PICKUP, GOING for 1st green");

			mbzirc_husky_msgs::Float64 srv;
			srv.request.data = 0;
			if (prepareClient.call(srv)){
				moveDistance = 0.4;	
				//						state = ROBOTALIGN_WITH_WALL_ODO;
				//active_storage = 0;
				active_layer++;
			}
		}
		else
		{
			state = ARMLOWPOSITIONING;
		}*/
	}

	ROS_INFO("FAILED TO STORE THE BRICK SUCCESSFULLY");
	return -1;
} 

int alignRobotWithBrick()
{
	behaviour = ROBOT_ALIGN_X_PHI; 
}

void actionServerCallback(const mbzirc_husky::brickPickupGoalConstPtr& goal, Server* as) 
{
	mbzirc_husky::brickPickupResult result;
	state = ARMRESET;//TODO
	EState nextState = state;
	EState recoveryState = state;
	while (isTerminal(state) == false && ros::ok()) 
	{
		printf("Active behaviour %s, active state %s\n",toStr(behaviour),toStr(state));
		if (behaviour == NONE){
			state = nextState;
			switch (state){
				case TEST1: if (moveRobot(+0.5) == 0) nextState = TEST2; else nextState = IDLE; break; 
				case TEST2: if (moveRobot(-0.5) == 0) nextState = TEST1; else nextState = IDLE; break;
				case ARMRESET: if (resetArm() == 0) nextState = ARMPOSITIONING; else nextState = ARMRESET; break;
				case ARMPOSITIONING: if (positionArm() == 0) {switchDetection(true);  nextState = ROBOT_ALIGNMENT;} else recoveryState = ARMPOSITIONING; break;
				case ARMPOSITIONING_NOMOVE: if (positionArm() == 0) {switchDetection(true);  nextState = ARMALIGNMENT;} else recoveryState = ARMPOSITIONING; break;
				case ROBOT_ALIGNMENT: alignRobotWithBrick(); nextState = ARMALIGNMENT; break;
				case ARMALIGNMENT: if (alignArm() == 0) nextState = ARMDESCENT; else nextState = ARMRESET; break;
				case ARMDESCENT: if (descentArm() == 0) nextState = ARMPICKUP; else nextState = ARMALIGNMENT; switchDetection(false); break;
				case ARMPICKUP:  if (pickupBrick() == 0) nextState = ARMSTORAGE; else nextState = ARMALIGNMENT; break;
				case ARMSTORAGE: if (prepareStorage() == 0) nextState = BRICKSTORE; else nextState = ARMALIGNMENT; break;
				case BRICKSTORE: if (storeBrick() == 0){
							 if (active_storage == 1)  nextState = ARMPOSITIONING_NOMOVE;
							 if (active_storage == 2)  nextState = ROBOT_MOVE_NEXT_BRICK;
							 if (active_storage == 3)  nextState = MOVE_TO_RED_BRICK_2;
						  }else { nextState = ARMRESET;} break;
				case ROBOT_MOVE_NEXT_BRICK: positionArm(); moveRobot(0.4); nextState = ROBOT_ALIGN_WITH_WALL; break;
				case ROBOT_ALIGN_WITH_WALL: switchDetection(true); alignRobotWithWall(0.05,NONE); nextState = MOVE_TO_GREEN_BRICK_1; break;
				case MOVE_TO_GREEN_BRICK_2: moveRobot(0.6); nextState = ARMRESET; break;
				case MOVE_TO_GREEN_BRICK_1: switchDetection(false); moveRobot(1.2); nextState = ARMPOSITIONING_NOMOVE; break;
				case MOVE_TO_RED_BRICK_1: switchDetection(false); moveRobot(-1.2); nextState = ARMPOSITIONING_NOMOVE; break;
			}
		}
		usleep(1200000);
	} 
if (state == ARMLOWPOSITIONING) {
			ROS_INFO("MOVING ARM INTO LOW BRICK POSITION");
			mbzirc_husky_msgs::Float64 srv;
			srv.request.data = -0.3;
			if (prepareClient.call(srv)) {
				usleep(3500000);
				state = ARMLOWALIGNMENT;
				mbzirc_husky_msgs::brickDetect brick_srv;
				brick_srv.request.activate            = true;
				brick_srv.request.groundPlaneDistance = 0;
				brick_srv.request.x                   = 640;
				brick_srv.request.y                   = 480;
				brickDetectorClient.call(brick_srv.request, brick_srv.response);

			} else {
				// unsafe
				//state = ROBOTALIGNMENT_PHI;
				// state = ARMALIGNMENT;
				ROS_INFO("ARM POSITION FAILED");
				usleep(3500000);
			}

		} else if (state == ARMLOWALIGNMENT) {
			usleep(5000000);
			mbzirc_husky_msgs::Float64 srv;
			srv.request.data = -0.3;
			if (alignClient.call(srv)) {
				state = ARMDESCENT;
				ROS_INFO("ARM ALIGNED");
			} else {
				usleep(500000);
				state = ARMLOWALIGNMENT;
				ROS_INFO("FAILED: FAILED TO ALIGN ARM (LOW)");
			}
		}
	usleep(1200000);

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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "brickPickup");
	ros::NodeHandle n;
	pn = &n;

	twistPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	brickDetectorClient = n.serviceClient<mbzirc_husky_msgs::brickDetect>("/detectBricks");
	prepareClient       = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/prepare_gripping");
	liftClient          = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/lift_brick");
	alignClient         = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/align_arm");
	pickupClient        = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/pickup_brick");
	homeClient          = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/home_arm");
	armStorageClient    = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/goto_storage");
	brickStoreClient    = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/store_brick");
	subscriberBrickPose = n.subscribe("/brickPosition", 1, &callbackBrickPose);
	subscriberScan = n.subscribe("/scan", 1, &scanCallBack);
	listener = new tf::TransformListener();
	// Dynamic reconfiguration server
	/*dynamic_reconfigure::Server<mbzirc_husky::sprayConfig> dynServer;
	  dynamic_reconfigure::Server<mbzirc_husky::sprayConfig>::CallbackType f = boost::bind(&callback, _1, _2);
	  dynServer.setCallback(f);*/

	server = new Server(n, "brickPickupServer", boost::bind(&actionServerCallback, _1, server), false);
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
