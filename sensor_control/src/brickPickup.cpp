#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <mbzirc_husky/brickPickupAction.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>

typedef actionlib::SimpleActionServer<mbzirc_husky::brickPickupAction> Server;
Server *server;

typedef enum{
	IDLE = 0,
	ROBOTALIGNMENT,
	ARMRESET,
    ARMPOSITIONING,
    ARMALIGNMENT,
    ARMDESCENT,
    ARMPICKUP,
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

bool isTerminal(ESprayState state)
{
    if(state == ROBOTALIGNMENT) return false;
    if(state == ARMRESET) return false;
    if(state == ARMPOSITIONING) return false;
    if(state == ARMALIGNMENT) return false;
    if(state == ARMDESCENT) return false;
    if(state == ARMPICKUP) return false;
    if(state == FINAL) return true;
	return true;
}

void actionServerCallback(const mbzirc_husky::brickPickupGoalConstPtr &goal, Server* as)
{
	mbzirc_husky::brickPickupResult result;

    state = ROBOTALIGNMENT;

    while (isTerminal(state) == false){
        if(state == ROBOTALIGNMENT)
        {
	    ROS_INFO("ALIGNING ROBOT");
            geometry_msgs::Twist msg;
            msg.linear.x = 0.1;
            twistPub.publish(msg);
	    usleep(1000000);
            msg.linear.x = -0.1;
            twistPub.publish(msg);
            msg.linear.x = 0.0;
            twistPub.publish(msg);
            state = ARMRESET;
	    ROS_INFO("ROBOT ALIGNED");
        }
	else if(state == ARMRESET)
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
                state = ARMALIGNMENT;
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
