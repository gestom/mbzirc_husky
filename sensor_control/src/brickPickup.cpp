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

typedef actionlib::SimpleActionServer<mbzirc_husky::brickPickupAction> Server;
Server *server;

typedef enum{
	IDLE = 0,
	ARMRESET,		//arm goes to dock position
	ARMPOSITIONING, 	//arm goes to overview positions
	ROBOTALIGNMENT_PHI,	  	//robot aligns to get the brick in nice position
	ROBOTALIGNMENT_X,	  	//robot aligns to get the brick in nice position
	ROBOTALIGNMENT_Y,	  	//robot aligns to get the brick in nice position
	ROBOTFINALALIGNMENT,	//robot aligns to get the brick in x direction only 
	ARMALIGNMENT,		//arm makes fine alignment
	ARMDESCENT,		//arm does down and detects the magnet feedback
	ARMPICKUP,		//grasp
	ARMSTORAGE,		//arm goes to position above the brick compartment
	BRICKSTORE,		//brick is put into the storage and magnet released
	ARMLOWPOSITIONING,
	ARMLOWALIGNMENT,	//arm makes fine alignment
	FINAL,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL
}ESprayState;
ESprayState state = IDLE;
ros::NodeHandle* pn;

ros::Publisher twistPub;

tf::TransformListener *listener;
geometry_msgs::PoseStamped anchorPose;

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

int active_storage = 0; // TODO make this an enum??;
int active_layer = 0; // TODO make this an enum??;
int incomingMessageCount = 0;

bool isTerminal(ESprayState state)
{
	if(state == ROBOTALIGNMENT_X) return false;
	if(state == ROBOTALIGNMENT_Y) return false;
	if(state == ROBOTALIGNMENT_PHI) return false;
	if(state == ROBOTFINALALIGNMENT) return false;
	if(state == ARMRESET) return false;
	if(state == ARMPOSITIONING) return false;
	if(state == ARMALIGNMENT) return false;
	if(state == ARMLOWPOSITIONING) return false;
	if(state == ARMLOWALIGNMENT) return false;
	if(state == ARMDESCENT) return false;
	if(state == ARMPICKUP) return false;
	if(state == ARMSTORAGE) return false;
	if(state == BRICKSTORE) return false;
	if(state == FINAL) return true;
	return true;
}

float stateMove = -1;
int alignmentOK = 0;

void callbackBrickPose(const mbzirc_husky_msgs::brickPositionConstPtr &msg) 
{
	int inc = 0;
	float maxX = 0.10;
	float maxZ = 0.15;
	float angle = tf::getYaw(msg->pose.pose.orientation);
	geometry_msgs::Twist spd;


	geometry_msgs::PoseStamped pose;
				geometry_msgs::PoseStamped tf_pose;
				geometry_msgs::PoseStamped robotPose;
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
					listener->waitForTransform("/base_link","map",pose.header.stamp,ros::Duration(0.1));
					listener->transformPose("/map",pose,robotPose);
				}
				catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
					//ros::Duration(1.0).sleep();
				}




	if(state == ROBOTFINALALIGNMENT)
	{
		printf("Robot final: %i %f %f %f\n",msg->detected,msg->pose.pose.position.x,msg->pose.pose.position.y,angle);
		spd.linear.x = -msg->pose.pose.position.x*3;
		spd.angular.z = 0;
		if (fabs(msg->pose.pose.position.x) < 0.02)
		{
			spd.linear.x = spd.linear.y = 0; 
			state = ROBOTFINALALIGNMENT;
		}
		if (spd.linear.x > +maxX) spd.linear.x = +maxX;
		if (spd.linear.x < -maxX) spd.linear.x = -maxX;
		if (spd.angular.z > +maxZ) spd.angular.z = +maxZ;
		if (spd.angular.z < -maxZ) spd.angular.z = -maxZ;
		if (fabs(msg->pose.pose.position.x) < 0.02)
		{
			alignmentOK++;
			if (alignmentOK > 20){
				spd.linear.x = spd.linear.y = 0; 
				state = ARMALIGNMENT;
			}		
		}else{
			alignmentOK = 0;
		}

		twistPub.publish(spd);
	}
	if (state == ROBOTALIGNMENT_PHI)
	{
		printf("Robot align: %i %f %f %f\n", msg->detected, msg->pose.pose.position.x, msg->pose.pose.position.y, angle);
		spd.linear.x = spd.angular.z = 0;
		if (msg->detected){
			if (incomingMessageCount++ > 90) {
				float angleDiff = (msg->pose.pose.position.y*3-angle);
				spd.angular.z =  angleDiff*10;
				//first, determine drone altitude, which is needed to check the expected size of objects
				if (fabs(angleDiff) < 0.01){
					anchorPose = robotPose;
				       	state = ROBOTALIGNMENT_Y; 
				}
				if (spd.linear.x > +maxX)  spd.linear.x = +maxX;
				if (spd.linear.x < -maxX)  spd.linear.x = -maxX;
				if (spd.angular.z > +maxZ) spd.angular.z = +maxZ;
				if (spd.angular.z < -maxZ) spd.angular.z = -maxZ;
			}
		}
		twistPub.publish(spd);
	}
	if (state == ROBOTALIGNMENT_Y)
	{
	if (state == ROBOTALIGNMENT_PHI && false) {
		ROS_INFO("ALIGNING ROBOT");
		spd.linear.x = spd.angular.z = 0;
		if (incomingMessageCount++ > 20){
			printf("Robot align: %i %f %f %f\n", msg->detected, msg->pose.pose.position.x, msg->pose.pose.position.y, angle);
			if (msg->detected) {
				spd.angular.z = (stateMove * msg->pose.pose.position.y * 3 - angle) * 20;
				if (fabs(angle) > 0.3 && spd.angular.z * angle > 0.0) spd.angular.z = 0;
				spd.linear.x = 0;
				if (msg->pose.pose.position.x > +0.1) stateMove = -1;
				if (msg->pose.pose.position.x < -0.1) stateMove = +1;
				if (angle * stateMove * msg->pose.pose.position.y > 0) spd.linear.x = stateMove * 0.10;
				if (fabs(msg->pose.pose.position.y) < 0.05 && fabs(angle) < 0.2) {
					alignmentOK++;
					if (alignmentOK > 20) {
						spd.linear.x = spd.linear.y = 0;
						state                       = ROBOTFINALALIGNMENT;
						twistPub.publish(spd);
						usleep(250000);
						alignmentOK = 0;
					}
				} else {
					alignmentOK = 0;
				}
			} else {
				spd.linear.x = spd.angular.z = 0;
			}
			if (spd.linear.x > +maxX)  spd.linear.x = +maxX;
			if (spd.linear.x < -maxX)  spd.linear.x = -maxX;
			if (spd.angular.z > +maxZ) spd.angular.z = +maxZ;
			if (spd.angular.z < -maxZ) spd.angular.z = -maxZ;
		}
		twistPub.publish(spd);
	}
}

void actionServerCallback(const mbzirc_husky::brickPickupGoalConstPtr& goal, Server* as) {
  mbzirc_husky::brickPickupResult result;

  state = ARMPOSITIONING;

  while (isTerminal(state) == false && ros::ok()) {
    if (state == ARMRESET) {
      ROS_INFO("RESETTING ARM INTO POSITION");
      std_srvs::Trigger srv;
      if (homeClient.call(srv)) {
        state = ARMPOSITIONING;
        ROS_INFO("ARM RESET");
      } else {
        // unsafe

        // state = ROBOTALIGNMENT;
        state = ARMALIGNMENT;
        ROS_INFO("ARM RESET FAILED");
      }

    } else if (state == ARMPOSITIONING) {
      ROS_INFO("MOVING ARM INTO POSITION");
      mbzirc_husky_msgs::Float64 srv;
      srv.request.data = 0;
      if (prepareClient.call(srv)) {
        usleep(3500000);
        state = ROBOTALIGNMENT_PHI;
	incomingMessageCount = 0; 
        mbzirc_husky_msgs::brickDetect brick_srv;
        brick_srv.request.activate            = true;
        brick_srv.request.groundPlaneDistance = 0;
        brick_srv.request.x                   = 640;
        brick_srv.request.y                   = 480;
        brickDetectorClient.call(brick_srv.request, brick_srv.response);
        // state = ARMALIGNMENT;
        ROS_INFO("ARM POSITIONED");
      } else {
        // unsafe
        state = ARMPOSITIONING;
        // state = ARMALIGNMENT;
        ROS_INFO("ARM POSITION FAILED");
      }
    } else if (state == ARMALIGNMENT) {
      ROS_INFO("ALIGNING ARM");
      mbzirc_husky_msgs::Float64 srv;
      srv.request.data = 0.0;
      if (alignClient.call(srv)) {
        usleep(3000000);
        state = ARMDESCENT;
        ROS_INFO("ARM ALIGNED");
      } else {
        state = ARMRESET;
        ROS_INFO("FAILED: FAILED TO ALIGN ARM SUCCESSFULLY");
      }
    } else if (state == ARMDESCENT) {
      ROS_INFO("ARM DESCENDING");
      std_srvs::Trigger srv;
      if (pickupClient.call(srv)) {
        ROS_INFO("ARM DESCENDED");
        state = ARMPICKUP;
        mbzirc_husky_msgs::brickDetect stop_brick_detection;
        stop_brick_detection.request.activate = false;
        brickDetectorClient.call(stop_brick_detection.request, stop_brick_detection.response);
      } else {
        ROS_INFO("FAILED TO DESCEND ARM, RE-ALIGNING");
        state = ARMRESET;
      }

    } else if (state == ARMPICKUP) {
      ROS_INFO("RAISING ARM");
      mbzirc_husky_msgs::Float64 srv;
      srv.request.data = 0;
      if (liftClient.call(srv)) {
        ROS_INFO("BRICK PICK UP DONE");
        state = ARMSTORAGE;
      } else {
        ROS_INFO("BRICK PICKUP FAILED");
        state = ARMALIGNMENT;
      }
    } else if (state == ARMSTORAGE) {
      ROS_INFO("MOVING ARM INTO STORAGE POSITION %d, LAYER %d", active_storage, active_layer);
      mbzirc_husky_msgs::StoragePosition srv;
      srv.request.position = active_storage;
      srv.request.layer    = active_layer;
      if (armStorageClient.call(srv)) {
        ROS_INFO("BRICK READY FOR STORAGE");
        state = BRICKSTORE;
      } else {
        ROS_INFO("FAILED TO REACH STORAGE");
      }
    } else if (state == BRICKSTORE) {
      ROS_INFO("STORING BRICK IN POSITION %d, LAYER %d", active_storage, active_layer);
      mbzirc_husky_msgs::StoragePosition srv;
      srv.request.position = active_storage;
      srv.request.layer    = active_layer;
      if (brickStoreClient.call(srv)) {
        ROS_INFO("BRICK STORED IN POSITION %d", active_storage);
        active_storage++;
        if (active_storage > 1) {
		ROS_INFO("FINISHED PICKUP");
		state = FINAL;

          //active_storage = 0;
          active_layer++;
        }
	else
	{
		state = ARMLOWPOSITIONING;
	}
      } else {
        ROS_INFO("FAILED TO STORE THE BRICK SUCCESSFULLY");
        state = ARMLOWPOSITIONING;
      }
    } else if (state == ARMLOWPOSITIONING) {
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
        state = ROBOTALIGNMENT_PHI;
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

	brickDetectorClient = n.serviceClient<mbzirc_husky_msgs::brickDetect>("/detectBricks");
	prepareClient       = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/prepare_gripping");
	liftClient          = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/lift_brick");
	alignClient         = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/align_arm");
	pickupClient        = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/pickup_brick");
	homeClient          = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/home_arm");
	armStorageClient    = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/goto_storage");
	brickStoreClient    = n.serviceClient<mbzirc_husky_msgs::StoragePosition>("/kinova/arm_manager/store_brick");
	subscriberBrickPose = n.subscribe("/brickPosition", 1, &callbackBrickPose);
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
