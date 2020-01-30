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

//int misdetections = 0;
ros::NodeHandle n;
ESprayState state = IDLE;
ros::Publisher cmd_vel;
geometry_msgs::Twist base_cmd;
ros::Subscriber thermalSubscriber; //only subscribe when in use: n.subscribe("/thermal/raw_temp_array", 1, thermalCallback);

CTimer timer;
CPump *pump;

int nThermalFrames = 0;
int nThermalFramesReq = 5;

/*Parameters we intend to make dynamically adjustable*/
float fwSpeed = 0.1f;
float rotationCoefficient = 40.0f; //bigger is slower
float fireThreshold = 45.0f;
int sprayTime = 1000; //in miliseconds

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
void thermalCallback(const std_msgs::String::ConstPtr& msg);
void startTurning()
{
  thermalSubscriber = n.subscribe("/thermal/raw_temp_array", 1, thermalCallback);
  state = TURNING;
  nThermalFrames = 0;
}

void stopTurning()
{
  thermalSubscriber.shutdown();
  state = APPROACHING;
}

void thermalCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Thermal message received");
  if(state != TURNING)
  {
    //rejecting image as we don't need it right not
    return;
  }
  
  int cameraWidth = 32;
  int cameraHeight = 32;
  int columnPeaks[cameraWidth];

  for(int colIdx = 0; colIdx < cameraWidth; colIdx++)
  {
    for(int rowIdx = 0; rowIdx < cameraHeight; rowIdx++)
    {
      int messageIndex = (rowIdx * cameraWidth) + colIdx;
      if((float)columnPeaks[colIdx] < (float)msg->data[messageIndex])
      {
        columnPeaks[colIdx] = (float)msg->data[messageIndex];      
      }
    }
  }

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
  int centredThresh = 2;
  if(peakIdx > (midPoint - centredThresh) && peakIdx < (midPoint + centredThresh)) 
  {
    if(nThermalFrames >= nThermalFramesReq)
    {
      stopTurning();
      return;
    }
    else
    {
      nThermalFrames++;
    }
  }

  float rotationalVelocity = -((peakIdx-midPoint) / rotationCoefficient);

  base_cmd.angular.z = rotationalVelocity;
  cmd_vel.publish(base_cmd);
}

void startSpraying()
{
  timer.reset();
  timer.start();
  state = SPRAYING;
  pump->on();
}

bool isTerminal(ESprayState state)
{
	if(state == TURNING) return false;
	if(state == APPROACHING) return false;
	if(state == ALIGNING) return false;
	if(state == SPRAYING) return false;
  if(state == FINAL) return true;
	return true;
}

void actionServerCallback(const mbzirc_husky::sprayGoalConstPtr &goal, Server* as)
{
	sprayTime = goal->duration;
	mbzirc_husky::sprayResult result;

  startTurning();

	/*if (state == IDLE){
		state = SPRAYING;
		timer.reset();
		timer.start();
		pump->on();
	}*/
	while (isTerminal(state) == false){
    if(state == TURNING)
    {
      //handled inside of themal callback
    }
    else if(state == APPROACHING)
    {
      //move towards target
      state = ALIGNING;
    }
    else if(state == ALIGNING)
    {
      //move arm to desired position
      startSpraying();
    }
		else if (state == SPRAYING && timer.getTime() > sprayTime){
     	state = FINAL;
			pump->off();
		}
		usleep(100);
	}
	pump->off();
	if (state == FINAL) state = SUCCESS; else state = FAIL;
	if (state == SUCCESS) 	server->setSucceeded(result);
	if (state == FAIL) 	server->setAborted(result);
	if (state == PREEMPTED) server->setPreempted(result);
	//state = STOPPING;
	
  base_cmd.linear.x = base_cmd.angular.z = 0;
	cmd_vel.publish(base_cmd);
	state = IDLE;
}


int main(int argc, char** argv)
{
	pump = new CPump("/dev/robot/pump");
	ros::init(argc, argv, "spray");
	//ros::NodeHandle n;
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
      pump->off();
      base_cmd.linear.x = base_cmd.angular.z = 0;
			cmd_vel.publish(base_cmd);
			state = IDLE;
		} 
		ros::spinOnce();
		usleep(30000);
	}
	delete pump;
}
