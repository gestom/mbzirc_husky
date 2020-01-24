#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
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
	IDLE = 0,
	TURNING,
	APPROACHING,
	ALIGNING,
	SPRAYING = 4,
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
ros::Subscriber thermalSubscriber;

CTimer timer;
CPump *pump;

/*Parameters we intend to make dynamically adjustable*/
float fwSpeed = 0.1f;
float rotationCoefficient = 30.0f; //bigger is slower
float fireThreshold = 45.0f;
int sprayTime = 4000; //in miliseconds

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

void thermalCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Thermal message received");


  if(!(state == TURNING ||
      state == APPROACHING ||
      state == ELEVATING ||
      state == SPRAYING))
  {
    //rejecting image
    return;
  }
  
  int cameraWidth = 32;
  int cameraHeight = 32;
  int columnPeaks[cameraWidth];

  /*for(int colIdx = 0; colIdx < cameraWidth; colIdx++)
  {
    for(int rowIdx = 0; rowIdx < cameraHeight; rowIdx++)
    {
      int messageIndex = (rowIdx * cameraWidth) + colIdx;
      if((float)columnPeaks[colIdx] < (float)msg->data[messageIndex])
      {
        columnPeaks[colIdx] = (float)msg->data[messageIndex];      
      }
    }
  }*/

  float peak = 0.0f;
  int peakIdx = -1;
  for(int i = 0; i < cameraWidth; i++)
  {
    if(columnPeaks[i] > peak)
    {
      peak = columnPeaks[i];
      peakIdx = i;
    }
  }

  if(peak < fireThreshold)
  {
    ROS_INFO("Thermal camera peak below camera threshold, stopping rot");
    base_cmd.angular.z = 0;
    return;
  }

  int midPoint = cameraWidth / 2;
  float rotationalVelocity = -((peakIdx-midPoint) / rotationCoefficient);

  base_cmd.angular.z = rotationalVelocity;
  cmd_vel.publish(base_cmd);
}

bool isTerminal(ESprayState state)
{
 	if (state == ALIGNING) return false;
 	if (state == SPRAYING) return false;
	return true;
}

void actionServerCallback(const mbzirc_husky::sprayGoalConstPtr &goal, Server* as)
{
	mbzirc_husky::sprayResult result;
	if (state == IDLE){
		state = SPRAYING;
		timer.reset();
		timer.start();
		pump->on();
	}
	while (isTerminal(state) == false){
		if (state == SPRAYING && timer.getTime() > sprayTime){
		       	state = FINAL;
			pump->off();
		}
		usleep(15000);
	}
	pump->off();
	if (state == FINAL) state = SUCCESS; else state = FAIL;
	if (state == SUCCESS) 	server->setSucceeded(result);
	if (state == FAIL) 	server->setAborted(result);
	if (state == PREEMPTED) server->setPreempted(result);
	state = STOPPING;
	state = IDLE;
}


int main(int argc, char** argv)
{
	pump = new CPump("/dev/robot/pump");
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
  ros::Subscriber sub = n.subscribe("/thermal/raw_temp_array", 1, thermalCallback);
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
	delete pump;
}
