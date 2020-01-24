#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <mbzirc_husky/sprayAction.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include "CPump.h"
#include "CTimer.h"
//#include <mbzirc_husky/sprayConfig.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef actionlib::SimpleActionServer<mbzirc_husky::sprayAction> Server;
Server *server;

typedef enum{
	IDLE,
	TURNING,
	APPROACHING,
	ALIGNING,
	SPRAYING,
	FINAL,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL
}ESprayState;

int misdetections = 0;
ESprayState state = IDLE;
ros::Publisher cmd_vel;
geometry_msgs::Twist base_cmd;
float fwSpeed = 0.1;
CTimer timer;

/*void callback(mbzirc_husky::sprayConfig &config, uint32_t level) {

	tolerance=config.tolerance;
	angleTolerance=config.angleTolerance;
	distanceTolerance=config.distanceTolerance;
	distance=config.distance;
	fwSpeed=config.fwSpeed;
	minPoints=config.minPoints;
	maxEvalu=config.maxEval;
	spdLimit=config.spdLimit;	
	realConst=config.realConst;
	dispConst=config.dispConst; 
	
}*/


bool isTerminal(ESprayState state)
{
 	if (state == ALIGNING) return false;
 	if (state == SPRAYING) return false;
 	if (state == FINAL) return false;
}

void actionServerCallback(const mbzirc_husky::sprayGoalConstPtr &goal, Server* as)
{
	mbzirc_husky::sprayResult result;
	state = SPRAYING;
	timer.reset();
	timer.start();
	while (isTerminal(state) == false){
		printf("%i\n",timer.getTime());
		if (state == FINAL) state = SUCCESS; else state = FAIL;
		usleep(15000);
	}
	if (state == SUCCESS) 	server->setSucceeded(result);
	if (state == FAIL) 	server->setAborted(result);
	if (state == PREEMPTED) server->setPreempted(result);
	state = STOPPING;
}


int main(int argc, char** argv)
{
	//CPump pump("/dev/robot/pump");
	CPump pump("/dev/ttyACM1");
	ros::init(argc, argv, "spray");
	ros::NodeHandle n;
	// Dynamic reconfiguration server
	/*dynamic_reconfigure::Server<mbzirc_husky::sprayConfig> dynServer;
  	dynamic_reconfigure::Server<mbzirc_husky::sprayConfig>::CallbackType f = boost::bind(&callback, _1, _2);
  	dynServer.setCallback(f);*/

	fwSpeed = 0.0;
	cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	server = new Server(n, "sprayServer", boost::bind(&actionServerCallback, _1, server), false);
	server->start();
	//scan_sub = n.subscribe("scan", 100, scanCallback);
	//robot_pose = n.subscribe("/robot_pose", 1000, poseCallback);
	//:point_pub_ = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/points2", 100000);
	while (ros::ok()){
		if (server->isPreemptRequested() && state != IDLE) state = PREEMPTED;
		if (state == STOPPING){
			base_cmd.linear.x = base_cmd.angular.z = 0;
			cmd_vel.publish(base_cmd);
			state = IDLE;
		} 
		ros::spinOnce();
		usleep(30000);
	}
}
