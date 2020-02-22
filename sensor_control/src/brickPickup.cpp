#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <sensor_msgs/LaserScan.h>
#include <mbzirc_husky/addInventory.h>
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
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr pcl_msg (new PointCloud);
PointCloud::Ptr pcl_two_line_msg (new PointCloud);
PointCloud::Ptr pcl_of_interest_msg (new PointCloud);
ros::Publisher point_pub;

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
	ROBOT_MOVE_SCAN,
	ROBOT_MOVE_ODO,
	BEHAVIOUR_NUMBER
}EBehaviour;

const char *behStr[] = { 
	"None",
	"aligning x and phi",
	"moving and turning to align y",
	"aligning to correct angle",
	"aligning along Y",
	"aligning along X",
	"moving along wall",
	"moving by odometry",
	"number"
};

EBehaviour behaviour = NONE;
EBehaviour nextBehaviour = NONE;
EBehaviour recoveryBehaviour = NONE;

float ransacTolerance = 0.05;
int behaviourResult = 0;
int robotXYMove = 1;
int alignMessageDelayCount = 30;
int velodynePacketCount = 0;
int num_bricks_desired = 1;

const char *stateStr[] = { 
	"Idle",
	"Approach 1",
	"Approach 2",
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
	APPROACH1,
	APPROACH2,
	ARMRESET,		//arm goes to dock position
	ARMPOSITIONING, 	//arm goes to overview positions
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
	MOVE_TO_BLUE_BRICK,	 //
	MOVE_TO_RED_BRICK_1,	 //
	MOVE_TO_RED_BRICK_2,	 //
	TERMINALSTATE,	 	 //marks terminal state
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
geometry_msgs::PoseStamped robotOdoPose;
geometry_msgs::Twist spd;
float anchorAngle = 0;
float wallAngleOffset = 0;

//service clients for the arm
ros::ServiceClient inventoryClient;
ros::ServiceClient service_client_brick_detector;
ros::ServiceClient prepareClient;
ros::ServiceClient liftClient;
ros::ServiceClient alignClient;
ros::ServiceClient pickupClient;
ros::ServiceClient homeClient;
ros::ServiceClient rearrangeClient;
ros::ServiceClient armStorageClient;
ros::ServiceClient brickStoreClient;
ros::ServiceClient brickDetectorClient;
ros::Subscriber subscriberBrickPose;
ros::Subscriber subscriberScan;
ros::Subscriber subscriberOdom;
ros::Subscriber velodyneSub;
ros::Publisher velodynePub;

int activeStorage = 0; // TODO make this an enum??;
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
	float maxX = 0.20;
	float maxZ = 0.15;
	if (speed.linear.x > +maxX) speed.linear.x = +maxX;
	if (speed.linear.x < -maxX) speed.linear.x = -maxX;
	if (speed.angular.z > +maxZ) speed.angular.z = +maxZ;
	if (speed.angular.z < -maxZ) speed.angular.z = -maxZ;
	twistPub.publish(speed);
}


int updateRobotPosition() {
	robotPose = robotOdoPose;
	return 0;
}

int updateRobotPositionold()
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
	printf("Movement done: %.3f %.3f\n",dist,moveDistance); 
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

int robotMTM(const sensor_msgs::LaserScanConstPtr &msg)
{
	spd.linear.x = spd.angular.z = 0;
	float dx = anchorPose.pose.position.x-robotPose.pose.position.x;
	float dy = anchorPose.pose.position.y-robotPose.pose.position.y;
	float dist = sqrt(dx*dx+dy*dy);
	spd.linear.x = (fabs(moveDistance) - dist + 0.1);
	if (moveDistance < 0) spd.linear.x = - spd.linear.x;
	if (dist > fabs(moveDistance) && moveDistance < 0){
		behaviour = nextBehaviour; 	
	}
	if (dist > moveDistance && moveDistance > 0)
	{
		spd.linear.x = 0;
		float angleDiff = anchorAngle-tf::getYaw(robotPose.pose.orientation);
		if (angleDiff > +M_PI) angleDiff-= 2*M_PI;
		if (angleDiff < -M_PI) angleDiff+= 2*M_PI;
		spd.angular.z =  angleDiff*10;
		if (fabs(angleDiff) < 0.01){
			anchorPose = robotPose;
			moveDistance = - moveDistance;
		}
	}
	setSpeed(spd);
	return 0;
}

void precise(float *ai,float *bi,float *x,float *y,int numPoints)
{
        float a = *ai;
        float b = *bi;
        float sxx,sxy,sx,sy;
        int n = 0;
        sxx=sxy=sx=sy=0;
        for (int j = 0; j <= numPoints; j++){
                if (fabs(sin(a)*x[j]+cos(a)*y[j]+b)<ransacTolerance){
                        sx += x[j];
                        sy += y[j];
                        sxx += x[j]*x[j];
                        sxy += x[j]*y[j];
                        n++;
                }
                if (fabs(sin(a)*x[j]+cos(a)*(y[j]+0.3)+b)<ransacTolerance){
                        sx += x[j];
                        sy += y[j]+0.3;
                        sxx += x[j]*x[j];
                        sxy += x[j]*(y[j]+0.3);
                        n++;
                }
        }
        if ((n*sxx-sx*sx) != 0 && n > 0){
                a = atan2(n*sxy-sx*sy,n*sxx-sx*sx);
                b = (sy-a*sx)/n;
                *ai=a;
                *bi=b;
        }
}




int robotMoveScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	spd.linear.x = spd.angular.z = 0;
	float dx = anchorPose.pose.position.x-robotPose.pose.position.x;
	float dy = anchorPose.pose.position.y-robotPose.pose.position.y;
	float dist = sqrt(dx*dx+dy*dy);


	size_t num_ranges = scan_msg->ranges.size();
	float x[num_ranges];
	float y[num_ranges];
	bool m[num_ranges];
	float angle;
	int numPoints = 0;
	float pX, pY;
	float maxX,minX,maxY,minY;
	minX = -0.0;
	maxX = +5.0;
	minY = -3.0;
	maxY = -0.4;
	for (int i = 0; i <= num_ranges; i++)
	{
		angle = scan_msg->angle_min+i*scan_msg->angle_increment;
		pX = scan_msg->ranges[i]*cos(angle);
		pY = scan_msg->ranges[i]*sin(angle);
		if (pX < maxX && pX > - maxX && pY > minY && pY < maxY){
			x[numPoints] = pX;
			y[numPoints] = pY;
			m[numPoints] = true;
			numPoints++;
		}
	}

    //stop box
    float stopMinX = -1.5;
    float stopMaxX = 0.0;
    float stopMinY = -3.0;
    float stopMaxY = -0.4;
    bool shouldStop = true;

    for(int i = 0; i < numPoints; i++)
    {
        if(x[i] < stopMaxX && x[i] > stopMinX && y[i] < stopMaxY && y[i] > stopMinY)
        {
            shouldStop = false;
            break;
        }
    }
    if(shouldStop)
    {
        spd.linear.x = spd.angular.z = 0;
        behaviour = nextBehaviour;
        printf("Movement done(gbox empty): %.3f %.3f\n",dist,moveDistance); 
        setSpeed(spd);
        //subscriberScan.shutdown();
        return 0;
    }

	int evalA,evalB = 0;
	int max_iterations = 100;
	float a,b;
	float maxA;
	float maxB;
	int maxEval,maxEvalA,maxEvalB;
	maxEval = maxEvalA = maxEvalB =0;
	for (int i = 0; i <= max_iterations; i++)
	{
		int aIndex = rand()%num_ranges;
		int bIndex = rand()%num_ranges;
		while (bIndex == aIndex) bIndex = rand()%num_ranges;
		if((x[bIndex]-x[aIndex]) == 0) continue;
		a = atan2(y[bIndex]-y[aIndex],x[bIndex]-x[aIndex]);
		b = y[bIndex]-sin(a)*x[bIndex];
		evalA = evalB = 0;
		for (int j = 0; j <= numPoints; j++){
			if (fabs(sin(a)*x[j]-cos(a)*y[j]+b)<ransacTolerance && m[j]) evalA++;
			if (fabs(sin(a)*x[j]-cos(a)*(y[j]+0.3)+b)<ransacTolerance && m[j]) evalB++;
		}
		if (maxEval < evalA+evalB){
			maxEval=evalA+evalB;
			maxEvalA = evalA;
			maxEvalB = evalB;
			maxA=a;
			maxB=b;
		}
	}
	int max_idx;
	precise(&maxA,&maxB,x,y,numPoints);
	if (maxA > M_PI/2){
	       	maxA = maxA - M_PI;
	       	maxB = -maxB;
	}
	if (maxA < - M_PI/2){
	       	maxA = maxA + M_PI;
	       	maxB = -maxB;
	}



	pcl_msg->header.frame_id = "velodyne";
	pcl_msg->height = 1;
	pcl_msg->points.clear();
	pcl_msg->width = 0;
	int b1 = -1;
	int b2 = -1;
	maxEvalA = 0;
	maxEvalB = 0;
	for (int j = 0; j <= numPoints; j++)
	{
		if (fabs(sin(maxA)*x[j]-cos(maxA)*y[j]+maxB)<ransacTolerance){
			pcl_msg->points.push_back (pcl::PointXYZ(x[j], y[j], 0.1));
			maxEvalA++;
			pcl_msg->width++;
		}
		if (fabs(sin(maxA)*x[j]-cos(maxA)*(y[j]+0.3)+maxB)<ransacTolerance){
			pcl_msg->points.push_back (pcl::PointXYZ(x[j], y[j], 0.1));
			maxEvalB++;
			pcl_msg->width++;
		}
	}
	pcl_conversions::toPCL(ros::Time::now(), pcl_msg->header.stamp);
	point_pub.publish (pcl_msg);
	spd.angular.z = 0;
	printf("Points: %i %i %i %.3f %.3f\n",numPoints,maxEvalA,maxEvalB,maxA,maxB);
	spd.linear.x = 0.1;
	float signMove = 1;
	if (moveDistance < 0) signMove = -1;

	if (maxEvalA + maxEvalB > 50) { spd.angular.z = maxA; spd.linear.x = 0.3;}
	if (maxEvalA + maxEvalB > 100 && maxB < -1.3 || maxEvalA > 50 && maxEvalB > 50) {spd.angular.z = signMove*(0.57+maxB);} 
	spd.linear.x = signMove*(fabs(moveDistance) - dist + 0.1);

	if (dist > fabs(moveDistance)) {
		spd.linear.x = spd.angular.z = 0;
		behaviour = nextBehaviour;
		printf("Movement done: %.3f %.3f\n",dist,moveDistance); 
		setSpeed(spd);
		//subscriberScan.shutdown();
		return 0;
	}
	setSpeed(spd);
	return 1;
}



int moveRobot(float distance,EBehaviour nextBeh = NONE)
{
	updateRobotPosition();
	anchorPose = robotPose;
	moveDistance = distance;
	printf("Move command:  %.3f\n",distance); 
	nextBehaviour = nextBeh;
	behaviour = ROBOT_MOVE_SCAN;
	return 0;
}

int moveTurnMove(float distance,EBehaviour nextBeh = NONE)
{
	moveDistance = distance;
	nextBehaviour = nextBeh;
	behaviour = ROBOT_MOVE_TURN_MOVE;
	return 0;
}

int alignRobotWithWall(float offset = 0,int messageDelayCount = 210,EBehaviour nb=NONE)
{
	wallAngleOffset = offset;
	alignMessageDelayCount = messageDelayCount;
	behaviour = ROBOT_ALIGN_PHI;
	nextBehaviour = nb;
}

int robotAlignXPhi(const mbzirc_husky_msgs::brickPositionConstPtr &msg)
{
	float angle = tf::getYaw(msg->pose.pose.orientation);
	geometry_msgs::Twist spd;
	spd.linear.x = spd.angular.z = 0;
	if (msg->detected){
		if (incomingMessageCount++ > alignMessageDelayCount) {
			float angleDiff = (msg->pose.pose.position.y*3-angle);
			spd.angular.z =  angleDiff*10;
			spd.linear.x = -msg->pose.pose.position.x;

			
			if (fabs(msg->pose.pose.position.y) < 0.08 && fabs(msg->pose.pose.position.x) < 0.02){
				spd.linear.x = spd.angular.z = 0;
				printf("Final robot alignment: %i %f %f %f\n", msg->detected, msg->pose.pose.position.x, msg->pose.pose.position.y, angle);
				anchorAngle = tf::getYaw(anchorPose.pose.orientation)-angle;
				alignRobotWithWall(robotXYMove*0.05,NONE); 
				return 0;
			}
			if ((fabs(angleDiff) < 0.01 && fabs(msg->pose.pose.position.x) < 0.02) || (angle*msg->pose.pose.position.y > 0 && fabs(angle) > 0.2)){
				printf("Current robot alignment: %i %f %f %f\n", msg->detected, msg->pose.pose.position.x, msg->pose.pose.position.y, angle);
				anchorPose = robotPose;
				anchorAngle = tf::getYaw(anchorPose.pose.orientation)-angle;
				printf("Anchor angle: %.3f %.3f %.3f\n",anchorAngle,tf::getYaw(anchorPose.pose.orientation),angle);
				moveTurnMove(0.3,ROBOT_ALIGN_X_PHI);
				return 1;
			}
		}
	}
	setSpeed(spd);
}

float uuu = 0;
float uux = 0;
float uuy = 0;
bool first = true;

void odoCallBack(const nav_msgs::OdometryConstPtr &msg) 
{
	/*this is exclusively for testing*/
	/*float aaa = tf::getYaw(msg->pose.pose.orientation);
	if (first){
	       	uuu = aaa;
		uux = msg->pose.pose.position.x;
		uuy = msg->pose.pose.position.y;
	}
	first = false;
	float dx = uux-msg->pose.pose.position.x;
	float dy = uuy-msg->pose.pose.position.y;
	float dist = sqrt(dx*dx+dy*dy);
	*/
	robotOdoPose.pose.position.x = msg->pose.pose.position.x;
	robotOdoPose.pose.position.y = msg->pose.pose.position.y;
	robotOdoPose.pose.position.z = 0;
	robotOdoPose.pose.orientation = msg->pose.pose.orientation;

	return;
}

int robotAlignPhi(const mbzirc_husky_msgs::brickPositionConstPtr &msg)
{
	float angle = tf::getYaw(msg->pose.pose.orientation);
	spd.linear.x = spd.angular.z = 0;
	if (msg->detected){
		if (incomingMessageCount++ > alignMessageDelayCount) {
			float angleDiff = wallAngleOffset-angle;
			spd.angular.z =  angleDiff*10;
			spd.linear.x = 0;
			if (fabs(angleDiff) < 0.01){
				printf("Robot aligned to the wall: %f\n", angle);
				behaviour = nextBehaviour;
				return 0;
			}
		}
	}
	setSpeed(spd);
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


void velodyneCallBack(const velodyne_msgs::VelodyneScanConstPtr &msg) 
{
	 velodynePub.publish(msg);
	 if (velodynePacketCount++ > 20) velodyneSub.shutdown();
}

void scanCallBack(const sensor_msgs::LaserScanConstPtr &msg) 
{
	if (updateRobotPosition() < 0) return;
	if (behaviour == ROBOT_MOVE_SCAN)  robotMoveScan(msg); 
	if (behaviour == ROBOT_MOVE_TURN_MOVE)  robotMTM(msg); 
	if (behaviour == ROBOT_MOVE_ODO)  robotMoveOdo(msg); 
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
	if (prepareClient.call(srv)) {
		ROS_INFO("ARM POSITIONED");
		return 0;
	}
	ROS_INFO("ARM POSITION FAILED");
	return -1;
}


int alignArm(bool high = true)
{
	ROS_INFO("ALIGNING ARM");
	//mbzirc_husky_msgs::Float64 srv;
	std_srvs::Trigger srv;
	//srv.request.data = 0.0;
	//if (activeStorage == 1) srv.request.data = -0.3;
	if (alignClient.call(srv)) {
		//usleep(3000000);
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

int prepareStorage()
{
	mbzirc_husky_msgs::StoragePosition srv = getStoragePosition(activeStorage);
	ROS_INFO("STORING BRICK IN POSITION %d, LAYER %d", srv.request.position , srv.request.layer);
	if (armStorageClient.call(srv)) {
		ROS_INFO("BRICK READY FOR STORAGE");
		return 0;
	}
	return -1;
}


int storeBrick()
{
	mbzirc_husky_msgs::StoragePosition srv = getStoragePosition(activeStorage);
  if (activeStorage + 1 == num_bricks_desired){ // next brick is the last one, do not raise arm after it is placed
    srv.request.keep_pressed = true;
  } else {
    srv.request.keep_pressed = srv.request.layer > 1;
  }
	if (brickStoreClient.call(srv)) {
		ROS_INFO("BRICK STORED IN POSITION %d", activeStorage);
		mbzirc_husky::addInventory inventSrv;
		inventSrv.request.position = srv.request.position;	
		inventSrv.request.layer = srv.request.layer;	
		if (inventSrv.request.position == 3) inventSrv.request.brickType = 2;
		if (inventSrv.request.position == 2) inventSrv.request.brickType = 1;
		if (inventoryClient.call(inventSrv)){
			ROS_INFO("INVENTORY UPDATED position %i and layer %i now has brick %i",inventSrv.request.position,inventSrv.request.layer,inventSrv.request.brickType);
		}else{
			ROS_INFO("INVENTORY UPDATE");
		}
		activeStorage++;
		return 0;
	}
	ROS_INFO("FAILED TO STORE THE BRICK SUCCESSFULLY");
	return -1;
} 

int pushBricks()
{
	ROS_INFO("REARRANGING BRICKS");
	std_srvs::Trigger srv;
	if (rearrangeClient.call(srv)) {
		ROS_INFO("BRICKS REARRANGED");
		return 0;
	}
	ROS_INFO("FAILED TO REARRANCGE BRICKS");
	return -1;
}



int descendStorage()
{
	mbzirc_husky_msgs::StoragePosition srv = getStoragePosition(activeStorage);
	ROS_INFO("REACHING FOR BRICK IN POSITION %d, LAYER %d", srv.request.position , srv.request.layer);
	if (armStorageClient.call(srv)) {
		ROS_INFO("BRICK ATTACHED");
		return 0;
	}
	return -1;
}

int alignRobotWithBrick()
{
	behaviour = ROBOT_ALIGN_X_PHI; 
}

int alignRobot()
{
	alignRobotWithBrick(); 
//	if (robotXYMove == 0) alignRobotWithBrick(); 
//	if (robotXYMove > 0) alignRobotWithWall(0.05); 
//	if (robotXYMove < 0) alignRobotWithWall(-0.05); 
}

bool shootVelodyne(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	velodynePacketCount = 0;
	velodyneSub = pn->subscribe("/velodyne_packets", 1, &velodyneCallBack);
	res.success = true;
	return true;
}



void actionServerCallback(const mbzirc_husky::brickPickupGoalConstPtr& goal, Server* as) 
{
  num_bricks_desired = goal->num_bricks_desired;
  activeStorage = 0;
ROS_INFO("[%s]: BRICK PICKUP STARTED. GOAL IS TO LOAD %d BRICKS", ros::this_node::getName().c_str(), num_bricks_desired);

	mbzirc_husky::brickPickupResult result;
	state = ARMRESET;	//TODO
	state = APPROACH1;	//TODO
	EState nextState = state;
	EState recoveryState = state;
	while (isTerminal(state) == false && ros::ok()) 
	{
		printf("Active behaviour %s, active state %s\n",toStr(behaviour),toStr(state));
		if (behaviour == NONE){
			state = nextState;
			switch (state){
				case APPROACH1: if (moveRobot(+2.5) == 0) nextState = APPROACH2; else nextState =  IDLE; break; 
				case APPROACH2: if (moveRobot(-2.5) == 0) nextState =  ARMRESET; else nextState = IDLE; break;
				case ARMRESET: switchDetection(false); if (resetArm() == 0) nextState = ARMPOSITIONING; else nextState = ARMRESET; break;
				case ARMPOSITIONING: if (positionArm() == 0) {switchDetection(true);  nextState = ROBOT_ALIGNMENT;} else recoveryState = ARMPOSITIONING; break;
				case ROBOT_ALIGNMENT: alignRobot(); nextState = ARMALIGNMENT; break;
				case ARMALIGNMENT: if (alignArm() == 0) nextState = ARMDESCENT; else nextState = ARMRESET; break;
				case ARMDESCENT: if (descentArm() == 0){ nextState = ARMPICKUP;} else nextState = ARMALIGNMENT; switchDetection(false); break;
				case ARMPICKUP:  if (pickupBrick() == 0) nextState = ARMSTORAGE; else nextState = ARMALIGNMENT; break;
				case ARMSTORAGE: if (prepareStorage() == 0) nextState = BRICKSTORE; else nextState = ARMPOSITIONING; break;
				case BRICKSTORE: if (storeBrick() == 0){
							 printf("STORAGE status %i\n",activeStorage);
               if (activeStorage == num_bricks_desired)
                 nextState = FINAL;
               else{
                // CHICKEN strategy - take 1 red on the first run, then take 3 reds and 2 greens on the second run
                if (activeStorage == 1)  {nextState = MOVE_TO_RED_BRICK_1;}	// one red brick is removed from before, so after a red is picked up, go straight for green 
							  if (activeStorage == 2)  {nextState = MOVE_TO_GREEN_BRICK_1;} // after picking up green, allow only backward phi alignment 							  
                if (activeStorage == 3)  {nextState = MOVE_TO_GREEN_BRICK_2;}	  // go back for two reds
							  if (activeStorage == 4)  {nextState = MOVE_TO_RED_BRICK_2;}
							  if (activeStorage == 5)  {nextState = FINAL;}

                 /* 
                //ORIGINAL greedy strategy - take 4 reds, 2 greens and 1 blue
							  if (activeStorage == 1)  {nextState = ARMPOSITIONING;}		//after the first red, only align along phi, then move forward
							  if (activeStorage == 2)  {nextState = MOVE_TO_GREEN_BRICK_1;} 		//after the second red, allow only forward phi alignment 
							  if (activeStorage == 3)  {nextState = MOVE_TO_GREEN_BRICK_2;}		//after picking up green, allow only backward phi alignment 
							  if (activeStorage == 4)  {nextState = MOVE_TO_RED_BRICK_2;}
							  if (activeStorage == 5)  {nextState = ARMPOSITIONING;}
							  if (activeStorage == 6)  {nextState = MOVE_TO_BLUE_BRICK;}
							  if (activeStorage == 7)  {nextState = FINAL;}
                */
                
              }
						 }else { nextState = ARMRESET;} break;

        /* 
        //ORIGINAL greedy strategy
				case MOVE_TO_GREEN_BRICK_1: switchDetection(false); moveRobot(1.75); robotXYMove = +1; positionArm();pushBricks(); nextState = ARMPOSITIONING; break;
				case MOVE_TO_GREEN_BRICK_2: moveRobot(0.7); robotXYMove = -1; nextState = ARMPOSITIONING; break;
				case MOVE_TO_RED_BRICK_2: switchDetection(false); moveRobot(-2.05); robotXYMove = +1; positionArm(); nextState = ARMPOSITIONING; break;
				case MOVE_TO_BLUE_BRICK: moveRobot(3.45); robotXYMove = +1; positionArm(); nextState = ARMPOSITIONING; break;
        */
        
        // CHICKEN strategy
        case MOVE_TO_RED_BRICK_1: moveRobot(0.4); robotXYMove = +1; nextState = ARMPOSITIONING; break;
        case MOVE_TO_GREEN_BRICK_1: switchDetection(false); moveRobot(1.35); robotXYMove = +1; positionArm(); pushBricks(); nextState = ARMPOSITIONING; break;
        case MOVE_TO_GREEN_BRICK_2: moveRobot(0.7); robotXYMove = -1; nextState = ARMPOSITIONING; break;
        case MOVE_TO_RED_BRICK_2: switchDetection(false); moveRobot(-2.05); robotXYMove = +1; positionArm(); nextState = ARMPOSITIONING; break;
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "brickPickup");
	ros::NodeHandle n;
	pn = &n;

	twistPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	subscriberScan = n.subscribe("/scanlocal", 1, &scanCallBack);
	subscriberOdom = n.subscribe("/odometry/filtered", 1, &odoCallBack);
	brickDetectorClient = n.serviceClient<mbzirc_husky_msgs::brickDetect>("/detectBricks");
	prepareClient       = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/prepare_gripping");
	liftClient          = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/lift_brick");
	alignClient         = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/align_arm");
	pickupClient        = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/pickup_brick");
	homeClient          = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/home_arm");
	rearrangeClient     = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/push_bricks");
	armStorageClient    = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/goto_storage");
	brickStoreClient    = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/store_brick");
	inventoryClient     = n.serviceClient<mbzirc_husky::addInventory>("/inventory/add");
	subscriberBrickPose = n.subscribe("/brickPosition", 1, &callbackBrickPose);
	point_pub = n.advertise<sensor_msgs::PointCloud2>("ransac/correct_one_line",10);
	velodynePub = n.advertise<velodyne_msgs::VelodyneScan>("/velodyne_packet_shot",10);
	ros::ServiceServer service = n.advertiseService("shootVelodyne", shootVelodyne);

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
