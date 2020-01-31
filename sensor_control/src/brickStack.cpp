#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <mbzirc_husky/brickStack.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>

typedef actionlib::SimpleActionServer<mbzirc_husky::brickStackAction> Server;
Server *server;

typedef enum{
	IDLE = 0,
	STACKING,
    FINAL,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL
}ESprayState;
ESprayState state = IDLE;
ros::NodeHandle* pn;

bool isTerminal(ESprayState state)
{
	if(state == STACKING) return false;
    if(state == FINAL) return true;
	return true;
}

void actionServerCallback(const mbzirc_husky::brickStackGoalConstPtr &goal, Server* as)
{
	mbzirc_husky::brickStackResult result;

    state = STACKING;

    while (isTerminal(state) == false){
        if(state == STACKING)
        {
            ROS_INFO("Stacking bricks");
            //handled inside of themal callback
            usleep(4000000);
            state = FINAL;
        }
        usleep(100);
    }
	if (state == FINAL) state = SUCCESS; else state = FAIL;
	if (state == SUCCESS) 	server->setSucceeded(result);
	if (state == FAIL) 	server->setAborted(result);
	if (state == PREEMPTED) server->setPreempted(result);
	//state = STOPPING;
    state = IDLE;	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "brickStack");
	ros::NodeHandle n;
    pn = &n;
	// Dynamic reconfiguration server
	/*dynamic_reconfigure::Server<mbzirc_husky::sprayConfig> dynServer;
  	dynamic_reconfigure::Server<mbzirc_husky::sprayConfig>::CallbackType f = boost::bind(&callback, _1, _2);
  	dynServer.setCallback(f);*/

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
