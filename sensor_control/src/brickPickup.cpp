#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <mbzirc_husky/brickPickupAction.h>
#include <mbzirc_husky_msgs/brickDetect.h>
#include <mbzirc_husky_msgs/brickPosition.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>

typedef actionlib::SimpleActionServer<mbzirc_husky::brickPickupAction> Server;
Server *server;

typedef enum{
	IDLE = 0,
	ARMRESET,		//arm goes to dock position
	ARMPOSITIONING, 	//arm goes to overview positions
	ROBOTALIGNMENT,	  	//robot aligns to get the brick in nice position
	ROBOTFINALALIGNMENT,	//robot aligns to get the brick in x direction only 
	ARMALIGNMENT,		//arm makes fine alignment
	ARMDESCENT,		//arm does down and detects the magnet feedback
	ARMPICKUP,		//grasp
	FINAL,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL
}ESprayState;
ESprayState state = IDLE;
ros::NodeHandle* pn;

ros::Publisher twistPub;

//service clients for the arm
ros::ServiceClient prepareClient;
ros::ServiceClient alignClient;
ros::ServiceClient pickupClient;
ros::ServiceClient homeClient;
ros::Subscriber subscriberBrickPose;

bool isTerminal(ESprayState state)
{
    if(state == ROBOTALIGNMENT) return false;
    if(state == ROBOTFINALALIGNMENT) return false;
    if(state == ARMRESET) return false;
    if(state == ARMPOSITIONING) return false;
    if(state == ARMALIGNMENT) return false;
    if(state == ARMDESCENT) return false;
    if(state == ARMPICKUP) return false;
    if(state == FINAL) return true;
	return true;
}

float stateMove = -1;
int alignmentOK = 0;

void callbackBrickPose(const mbzirc_husky_msgs::brickPositionConstPtr &msg) 
{
	float maxX = 0.10;
	float maxZ = 0.15;
	float angle = tf::getYaw(msg->pose.pose.orientation);
	geometry_msgs::Twist spd;
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
	if(state == ROBOTALIGNMENT){
		printf("Robot align: %i %f %f %f\n",msg->detected,msg->pose.pose.position.x,msg->pose.pose.position.y,angle);
		if (msg->detected){	
			spd.linear.x = -msg->pose.pose.position.x*3;
			//if (spd.linear.x < 0) spd.angular.z = -msg->pose.pose.position.y;
			//if (spd.linear.x > 0) spd.angular.z = msg->pose.pose.position.y;
			spd.angular.z = (stateMove*msg->pose.pose.position.y*3-angle)*20;
			if (fabs(angle) > 0.3 && spd.angular.z*angle > 0.0) spd.angular.z = 0;
			spd.linear.x = 0;
			if (msg->pose.pose.position.x > +0.1) stateMove = -1;
			if (msg->pose.pose.position.x < -0.1) stateMove = +1;
			if (angle*stateMove*msg->pose.pose.position.y > 0) spd.linear.x = stateMove*0.10;

		}else{
			spd.linear.x = spd.linear.y = 0; 
		}
		if (fabs(msg->pose.pose.position.y) < 0.02 && fabs(angle) < 0.2)
		{
			alignmentOK++;
			if (alignmentOK > 20){
				spd.linear.x = spd.linear.y = 0; 
				state = ROBOTFINALALIGNMENT;
				twistPub.publish(spd);
				usleep(250000);
				alignmentOK=0;
			}
		}else{
			alignmentOK=0;
		}
		if (spd.linear.x > +maxX) spd.linear.x = +maxX;
		if (spd.linear.x < -maxX) spd.linear.x = -maxX;
		if (spd.angular.z > +maxZ) spd.angular.z = +maxZ;
		if (spd.angular.z < -maxZ) spd.angular.z = -maxZ;
		twistPub.publish(spd);
	}
}

void actionServerCallback(const mbzirc_husky::brickPickupGoalConstPtr &goal, Server* as)
{
	mbzirc_husky::brickPickupResult result;

	state = ARMRESET;

	while (isTerminal(state) == false && ros::ok()){
		if(state == ARMRESET)
		{
			ROS_INFO("RESETTING ARM INTO POSITION");
			std_srvs::Trigger srv;
			if(homeClient.call(srv))
			{
				state = ARMPOSITIONING;
				ROS_INFO("ARM RESET");
			}
			else
			{
				//unsafe
				state = ROBOTALIGNMENT;
				ROS_INFO("ARM RESET FAILED");
			}

		}
		else if(state == ARMPOSITIONING)
		{
			ROS_INFO("MOVING ARM INTO POSITION");
			std_srvs::Trigger srv;
			if(prepareClient.call(srv))
			{
				usleep(3000000);
				state = ROBOTALIGNMENT;
				ROS_INFO("ARM POSITIONED");
			}
			else
			{
				//unsafe
				state = ROBOTALIGNMENT;
				ROS_INFO("ARM POSITION FAILED");
			}
		}
		else if(state == ARMALIGNMENT)
		{
			ROS_INFO("ALIGNING ARM");
			std_srvs::Trigger srv;
			if(alignClient.call(srv))
			{
				state = ARMDESCENT;
				ROS_INFO("ARM ALIGNED");
			}
			else
			{
				state = FAIL;
				ROS_INFO("FAILED: FAILED TO ALIGN ARM");
			}
		}
		else if(state == ARMDESCENT)
		{
			ROS_INFO("ARM DESCENDING");
			std_srvs::Trigger srv;
			if(pickupClient.call(srv))
			{
				ROS_INFO("ARM DESCENDED");
				state = ARMPICKUP;
			}
			else
			{
				ROS_INFO("FAILED TO DESCEND ARM, RE-ALIGNING");
				state = ARMALIGNMENT;
			}
		}
		else if(state == ARMPICKUP)
		{
			ROS_INFO("RAISING ARM");
			std_srvs::Trigger srv;
			if(prepareClient.call(srv))
			{
				ROS_INFO("BRICK PICK UP DONE");
				state = FINAL;
			}
			else
			{
				ROS_INFO("BRICK PICKUP FAILED");
				state = FAIL;
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "brickPickup");
    ros::NodeHandle n;
    pn = &n;

    twistPub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    prepareClient = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/prepare_gripping");
    alignClient = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/align_arm");
    pickupClient = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/pickup_brick");
    homeClient = n.serviceClient<std_srvs::Trigger>("/kinova/arm_manager/home_arm");
    subscriberBrickPose   = n.subscribe("/brickPosition", 1, &callbackBrickPose);

    // Dynamic reconfiguration server
    /*dynamic_reconfigure::Server<mbzirc_husky::sprayConfig> dynServer;
      dynamic_reconfigure::Server<mbzirc_husky::sprayConfig>::CallbackType f = boost::bind(&callback, _1, _2);
      dynServer.setCallback(f);*/

    server = new Server(n, "brickPickupServer", boost::bind(&actionServerCallback, _1, server), false);
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
