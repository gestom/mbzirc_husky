#include <ros/ros.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <mbzirc_husky/boundsConfig.h>
#include <mbzirc_husky/setPoi.h>
#include <mbzirc_husky/getPoi.h>

#include <vector>
#include <opencv2/core/types.hpp>
#include <iostream>
#include <fstream>

ros::NodeHandle* pn;

double map_width;
double map_height;
double map_x;
double map_y;
double map_rotation;

std::vector<cv::Point2d> waypoints;

void callback(mbzirc_husky::boundsConfig &config, uint32_t level) {
        map_width=config.w;
        map_height=config.h;
        map_x=config.x;
        map_y=config.y;
        map_rotation=config.r;
}

/* service for set/reset the distance */
bool setPointCallback(mbzirc_husky::setPoi::Request &req, mbzirc_husky::setPoi::Response &res)
{
        ROS_INFO("Setting point for the symbolic map");
        res.res = true;
	return true;

}

bool getPointCallback(mbzirc_husky::getPoi::Request &req, mbzirc_husky::getPoi::Response &res)
{
        ROS_INFO("Retrieving point from the symbolic map");
	float x= 0;
	float y= 0;
       	res.x = x;
	res.y = y;
	res.type = req.type;
        return true;
}

int loadWaypoints(){
	std::ifstream loadFile;
	loadFile.open("./src/mbzirc_husky/sensor_control/bricks/bricks.txt");
	int i=0;
	float x,y;
	while(loadFile >> x >> y)
	{
		cv::Point2d tmp;
		tmp.x = x;
		tmp.y = y;
		waypoints.push_back(tmp);
	}
	loadFile.close();
	return waypoints.size();	
}

	



int main(int argc, char** argv)
{
	ros::init(argc, argv, "symbolicMap");
	ros::NodeHandle n;
	pn = &n;

 	// service servers
	ros::ServiceServer set_map_srv = n.advertiseService("set_map_poi", setPointCallback);
	ros::ServiceServer get_map_srv = n.advertiseService("get_map_poi", getPointCallback);

	// Dynamic reconfiguration server
	dynamic_reconfigure::Server<mbzirc_husky::boundsConfig> dynServer;
	dynamic_reconfigure::Server<mbzirc_husky::boundsConfig>::CallbackType f = boost::bind(&callback, _1, _2);
	dynServer.setCallback(f);

	ros::spin();

}

