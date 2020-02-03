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
            geometry_msgs::Twist msg;
            msg.linear.x = 0.1;
            twistPub.publish(msg);
            msg.linear.x = -0.1;
            twistPub.publish(msg);
            state = ARMPOSITIONING;
        }
        else if(state == ARMPOSITIONING)
        {
            std_srvs::Trigger srv;
            if(prepareClient.call(srv))
            {
                state = ARMALIGNMENT;
            }
            else
            {
                //unsafe
                state = ROBOTALIGNMENT;
            }
        }
        else if(state == ARMALIGNMENT)
        {
            std_srvs::Trigger srv;
            if(alignClient.call(srv))
            {
                state = ARMDESCENT;
            }
            else
            {
                state = FAIL;
            }
        }
        else if(state == ARMDESCENT)
        {
            std_srvs::Trigger srv;
            if(pickupClient.call(srv))
            {
                state = ARMPICKUP;
            }
            else
            {
                state = ARMALIGNMENT;
            }
        }
        else if(state == ARMPICKUP)
        {
            std_srvs::Trigger srv;
            if(prepareClient.call(srv))
                state = FINAL;
            else
                state = FAIL;
        }
        usleep(100);
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
