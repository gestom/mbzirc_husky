#include <ros/ros.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <tf/transform_listener.h>
#include <mbzirc_husky/boundsConfig.h>
#include <mbzirc_husky/symbolicMapConfig.h>
#include <mbzirc_husky/setPoi.h>
#include <mbzirc_husky/getPoi.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <opencv2/core/types.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include "order.h"

ros::NodeHandle* pn;

ros::Publisher waypointVisualiser;
ros::Publisher waypointRangeVisualiser;

//Map reconfigure
double map_width;
double map_height;
double map_x;
double map_y;
double map_rotation;

int waypointIdx = 0;
float maxObservationDistance = 4.0f;
int hypothesisIdx[8];

//Clustering reconfigure
double tolerance = 0.5;

std::vector<cv::Point2d> waypoints;
std::vector<std::vector<cv::Point2d>> robotDelivery;
std::vector<std::vector<cv::Point2d>> dronePile;
std::vector<std::vector<cv::Point2d>> droneDelivery;
std::vector<std::vector<cv::Point2d>> redBricks;
std::vector<std::vector<cv::Point2d>> blueBricks;
std::vector<std::vector<cv::Point2d>> greenBricks;
std::vector<std::vector<cv::Point2d>> orangeBricks;

void publishWaypoints();
void organisePath();

void callback(mbzirc_husky::boundsConfig &config, uint32_t level) {
        map_width=config.w;
        map_height=config.h;
        map_x=config.x;
        map_y=config.y;
        map_rotation=config.r;
}

void clusterCallback(mbzirc_husky::symbolicMapConfig &config, uint32_t level) {
        tolerance=config.tolerance;
}


cv::Point2d getCenter(std::vector<cv::Point2d> cluster){
	cv::Point2d res;
	res.x = 0;
	res.y = 0;
	int c_size=0;
	for (int i =0; i < cluster.size();i++){
		res.x+=cluster[i].x;
		res.y+=cluster[i].y;
		c_size++;
	}
	//Center Point
	res.x= res.x/c_size;	
	res.y= res.y/c_size;
	return res;
}


double dist(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

bool distanceToCenter(std::vector<cv::Point2d> cluster,cv::Point2d query){
	cv::Point2d center;
	
	center = getCenter(cluster);

	if(dist(center.x,center.y,query.x,query.y) <= tolerance)return true;
       	else  return false;	
}	



// Take incoming point from service
// Point type - 0 waypoint, 1 Drone Pile, 2 Drone delivery, 3 robot delivery, 4 red bricks, 5 green, 6 blue, 7 orange 
bool setPointCallback(mbzirc_husky::setPoi::Request &req, mbzirc_husky::setPoi::Response &res)
{
	int type;
	cv::Point2d point;
	point.x = req.x;
	point.y = req.y;
	std::vector<cv::Point2d> vPoint;
	vPoint.clear();
	vPoint.push_back(point);
 	switch (req.type){
		case 0: ROS_INFO("Setting point for the waypoints symbolic map at position X: %f Y: %f", point.x,point.y);
			type = req.type;
			waypoints.push_back(point);
			res.res = true;
			return true;
			break;
		case 1: type = req.type;
			//First Point of a type 
			if(dronePile.empty()) {
				dronePile.push_back(vPoint);
				ROS_INFO("Setting point for the Drone Pile in symbolic mapc at position X: %f Y: %f",point.x,point.y);
				res.res = true;
				return true;
			} else {
				//Test Distance to center of all same points to all clusters
				//If not close to any cluster create a new one 
				for(int i = 0; i < dronePile.size();i++){

					if(distanceToCenter(dronePile[i],point)){
						dronePile[i].push_back(point);	
						ROS_INFO("Setting point for the Drone Pile in symbolic map at position X: %f Y: %f , cluster %i ",point.x,point.y,i);
						res.res = true;
						return true;
					}
				}
				ROS_INFO ("Point for Drone Pile way to far from previous at position X: %f Y: %f setting up new cluster", point.x,point.y);	
				dronePile.push_back(vPoint);
				res.res = true;
				return true;	

			}
			break;
		case 2: if(droneDelivery.empty()) {
				droneDelivery.push_back(vPoint);
				ROS_INFO("Setting first point for the Drone Delivery in symbolic map at position X: %f Y: %f", point.x,point.y);
				res.res = true;
				return true;
			} else {
				for(int i = 0; i < droneDelivery.size();i++){

					if(distanceToCenter(droneDelivery[i],point)){
						droneDelivery[i].push_back(point);	
						ROS_INFO("Setting point for the Drone Delivery in symbolic map at position X: %f Y: %f cluster %i", point.x,point.y,i);
						res.res = true;
						return true;
					}
				}	
				ROS_INFO ("Point for Drone Delivery way to far from previous at position X: %f Y: %f setting up new cluster ", point.x,point.y);
				dronePile.push_back(vPoint);
				res.res = true;
				return true;	
			}
			break;
		case 3: if(robotDelivery.empty()) {
				robotDelivery.push_back(vPoint);
				ROS_INFO("Setting first point for the Robot Delivery in symbolic map at position X: %f Y: %f", point.x,point.y);
				res.res = true;
				return true;
			} else {
				for(int i = 0; i < robotDelivery.size();i++){
					if(distanceToCenter(robotDelivery[i],point)){
						robotDelivery[i].push_back(point);	
						ROS_INFO("Setting point for the Robot Delivery in symbolic map at position X: %f Y: %f cluster %i",point.x,point.y, i);
						res.res = true;
						return true;
					}
				}
				ROS_INFO ("Point for Robot Delivery way to far from previous at position X: %f Y: %f setting up new cluster",point.x,point.y);
				robotDelivery.push_back(vPoint);
				res.res = true;
				return true;	
			}		
			break;
		case 4: if(redBricks.empty()) {
				redBricks.push_back(vPoint);
				ROS_INFO("Setting first point for the Red Bricks in  symbolic map at position X: %f Y: %f ", point.x,point.y);
				res.res = true;
				return true;
			} else {
				for(int i = 0; i < redBricks.size();i++){
					if(distanceToCenter(redBricks[i],point)){
						redBricks[i].push_back(point);	
						ROS_INFO("Setting point for the Red Bricks in  symbolic map at position X: %f Y: %f cluster %i", point.x,point.y,i);
						res.res = true;
						return true;
					}
				}
				ROS_INFO ("Point for Red Bricks way to far from previous at position X: %f Y: %f setting up new cluster", point.x,point.y);
				redBricks.push_back(vPoint);
				res.res = true;
				return true;	
			}
			break;
		case 5: if(greenBricks.empty()) {
				greenBricks.push_back(vPoint);
				ROS_INFO("Setting first point for the Green Bricks in symbolic map at position X: %f Y: %f",point.x,point.y);
				res.res = true;
				return true;
			} else {
				for(int i = 0; i < greenBricks.size();i++){
					if(distanceToCenter(greenBricks[i],point)){
						greenBricks[i].push_back(point);	
						ROS_INFO("Setting point for the Green Bricks in symbolic map at position X: %f Y: %f cluster %i ",point.x,point.y,i);
						res.res = true;
						return true;
					}
				}
				ROS_INFO ("Point for Green Bricks way to far from previous at position X: %f Y: %f setting up new cluster", point.x,point.y);
				greenBricks.push_back(vPoint);
				res.res = true;
				return true;	
			}
			break;
		case 6: if(blueBricks.empty()) {
				blueBricks.push_back(vPoint);
				ROS_INFO("Setting first point for the Blue Bricks in symbolic map at position X: %f Y: %f",point.x,point.y);
				res.res = true;
				return true;
			} else {
				for(int i = 0; i < blueBricks.size();i++){
					if(distanceToCenter(blueBricks[i],point)){
						blueBricks[i].push_back(point);	
						ROS_INFO("Setting point for the Blue Bricks in symbolic map at position X: %f Y: %f cluster %i",point.x,point.y,i);
						res.res = true;
						return true;
					}
				}		
				ROS_INFO ("Point for Blue Bricks way to far from previous at position X: %f Y: %f setting up new cluster",point.x,point.y);
				blueBricks.push_back(vPoint);
				res.res = true;
				return true;	
			}
			break;
		case 7: if(orangeBricks.empty()) {
				orangeBricks.push_back(vPoint);
				ROS_INFO("Setting first point for the orange Bricks in  symbolic map at position X: %f Y: %f",point.x,point.y);
				res.res = true;
				return true;
			} else {
				for(int i = 0; i < orangeBricks.size();i++){
					if(distanceToCenter(orangeBricks[i],point)){
						orangeBricks[i].push_back(point);	
						ROS_INFO("Setting point for the Orange Bricks symbolic map at position X: %f Y: %f cluster %i", point.x,point.y,i);
						res.res = true;
						return true;
					}
				}		
				ROS_INFO ("Point for Orange Bricks way to far from previous at position X: %f Y: %f setting up new cluster",point.x,point.y);
				orangeBricks.push_back(vPoint);
				res.res = true;
				return true;	
			}
			break;
	}
	ROS_INFO("Wrong input data");
	res.res = false;
	return false;

}

bool getPointCallback(mbzirc_husky::getPoi::Request &req, mbzirc_husky::getPoi::Response &res){
	int incoming_type = req.type;
	cv::Point2d point;

	ROS_INFO("Incomin service of type %i ", req.type);	
	switch (incoming_type){
		case 0: if(!waypoints.empty()){
                		publishWaypoints();
				point = waypoints[waypointIdx];
				res.x.push_back(point.x);
				res.y.push_back(point.y);
				res.type = incoming_type;
				waypointIdx++;
				if(waypointIdx > (waypoints.size()-1)) waypointIdx = 0; 
				ROS_INFO("Retrieving the next point from the waypoints symbolic map, point type %i at position X: %f Y: %f", req.type,point.x,point.y);
				return true;
			}
			else {
				ROS_INFO("EMPTY MAP FOR TYPE %i! ",req.type);
				return false;
			}	
			break;
		case 1: if(!dronePile.empty()){
				for(int i = 0; i < dronePile.size();i++){
					point = getCenter(dronePile[i]);
					res.x.push_back(point.x);
					res.y.push_back(point.y);
					res.type = incoming_type;
					ROS_INFO("Retrieving center point for the Drone Pile from symbolic map, point type %i at position X: %f Y: %f for cluster %i", req.type,point.x,point.y,i);
				}
				return true;

			}
			else {
				ROS_INFO("EMPTY MAP FOR TYPE %i! ",req.type);
				return false;
			}	

			break;
		case 2:	if(!droneDelivery.empty()){
				for(int i = 0; i < droneDelivery.size();i++){
					point = getCenter(droneDelivery[i]);
					res.x.push_back(point.x);
					res.y.push_back(point.y);
					res.type = incoming_type;
					ROS_INFO("Retrieving point for the Drone Delivery symbolic map, point type %i at position X: %f Y: %f", req.type,point.x,point.y);
				}
				return true;
			}
			else {
				ROS_INFO("EMPTY MAP FOR TYPE %i! ",req.type);
				return false;
			}	

			break;
		case 3: if(!robotDelivery.empty()){
				for(int i = 0; i < robotDelivery.size();i++){
					point = getCenter(robotDelivery[i]);
					res.x.push_back(point.x);
					res.y.push_back(point.y);
					res.type = incoming_type;
					ROS_INFO("Retrieving point for the Robot Delivery symbolic map, point type %i at position X: %f Y: %f", req.type,point.x,point.y);
				}
				return true;
			}
			else {
				ROS_INFO("EMPTY MAP FOR TYPE %i! ",req.type);
				return false;
			}	

			break;
		case 4: if(!redBricks.empty()){
				for(int i = 0; i < redBricks.size();i++){
					point = getCenter(redBricks[i]);
					res.x.push_back(point.x);
					res.y.push_back(point.y);
					res.type = incoming_type;
					ROS_INFO("Retrieving point for the Red Bricks symbolic map, point type %i at position X: %f Y: %f", req.type,point.x,point.y);
				}
				return true;
			}
			else {
				ROS_INFO("EMPTY MAP FOR TYPE %i! ",req.type);
				return false;
			}	

			break;
		case 5: if(!greenBricks.empty()){
				for(int i = 0; i < greenBricks.size();i++){
					point = getCenter(greenBricks[i]);
					res.x.push_back(point.x);
					res.y.push_back(point.y);
					res.type = incoming_type;
					ROS_INFO("Retrieving  point for the Green Bricks symbolic map, point type %i at position X: %f Y: %f", req.type,point.x,point.y);
				}
				return true;
			}
			else {
				ROS_INFO("EMPTY MAP FOR TYPE %i! ",req.type);
				return false;
			}	

			break;
		case 6:	if(!blueBricks.empty()){
				for(int i = 0; i < blueBricks.size();i++){
					point = getCenter(blueBricks[i]);
					res.x.push_back(point.x);
					res.y.push_back(point.y);
					res.type = incoming_type;
					ROS_INFO("Retrieving point for the Blue Bricks symbolic map, point type %i at position X: %f Y: %f", req.type,point.x,point.y);
				}
				return true;
			}
			else {
				ROS_INFO("EMPTY MAP FOR TYPE %i! ",req.type);
				return false;
			}	

			break;
		case 7: if(!orangeBricks.empty()){
				for(int i = 0; i < orangeBricks.size();i++){
					point = getCenter(orangeBricks[i]);
					res.x.push_back(point.x);
					res.y.push_back(point.y);
					res.type = incoming_type;
					ROS_INFO("Retrieving point for the Orange symbolic map, point type %i at position X: %f Y: %f", req.type,point.x,point.y);
				}
				return true;
			}
			else {
				ROS_INFO("EMPTY MAP FOR TYPE %i! ",req.type);
				return false;
			}	
			break;
	}
	ROS_INFO("Wrong data format for obtaining data from symbolic map");
	return false;	
}

void loadWaypoints(){
	const char* filename = (char*) "./src/mbzirc_husky/sensor_control/maps/tennis-no-bricks/map-waypoints.txt";

    ROS_INFO("Opening waypoint file: %s", filename);
    std::ifstream loadFile(filename);

	float x,y;
	while(loadFile >> x >> y){
		cv::Point2d tmp;
		tmp.x = x;
		tmp.y = y;
		waypoints.push_back(tmp);
	}
	ROS_INFO("Loaded waypoints with %i points",(int)waypoints.size());
	loadFile.close();
    	organisePath();
	publishWaypoints();
}

void organisePath()
{
	float* xs = new float[waypoints.size()];
	float* ys = new float[waypoints.size()];

	for(int i = 0; i < waypoints.size(); i++)
	{
		xs[i] = waypoints[i].x;
		ys[i] = waypoints[i].y;
		printf("%f %f\n", xs[i], ys[i]);
	}

	CTSP pf(xs, ys, waypoints.size());
	pf.solve(3);
	pf.print();
	delete[] xs;
	delete[] ys;
}

void publishWaypoints()
{
    ROS_INFO("Publishing waypoints");
    for(int i = 0; i < waypoints.size(); i++)
    {
        visualization_msgs::Marker msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.id = i;
        msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        msg.pose.position.x = waypoints[i].x;
        msg.pose.position.y = waypoints[i].y;
        msg.pose.position.z = 0.2;

        msg.color.a = 1;
        msg.color.r = 1;
        msg.color.g = 0;
        msg.color.b = 1;

        msg.scale.z = 1;
       
       	std::ostringstream stream;
        stream << (i - waypointIdx) % waypoints.size() ;
     	msg.text = stream.str();	
	waypointVisualiser.publish(msg);

        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.type = visualization_msgs::Marker::CYLINDER;

        msg.pose.position.x = waypoints[i].x;
        msg.pose.position.y = waypoints[i].y;
        msg.pose.position.z = 0;

        msg.color.a = 1;
        msg.color.r = 1;
        msg.color.g = 1;
        msg.color.b = 0;

	msg.scale.x = maxObservationDistance*2;
	msg.scale.y = maxObservationDistance*2;
        msg.scale.z = 0.01;
       
	waypointRangeVisualiser.publish(msg);

    }
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "symbolicMap");
	ros::NodeHandle n;
	pn = &n;

    waypointVisualiser = n.advertise<visualization_msgs::Marker>("/symbolicMap/waypoints", 100);
    waypointRangeVisualiser = n.advertise<visualization_msgs::Marker>("/symbolicMap/waypointRanges", 100);

	// service servers
	ros::ServiceServer set_map_srv = n.advertiseService("set_map_poi", setPointCallback);
	ros::ServiceServer get_map_srv = n.advertiseService("get_map_poi", getPointCallback);

	// Dynamic reconfiguration server
	dynamic_reconfigure::Server<mbzirc_husky::boundsConfig> dynServer;
	dynamic_reconfigure::Server<mbzirc_husky::boundsConfig>::CallbackType f = boost::bind(&callback, _1, _2);
	dynServer.setCallback(f);

	dynamic_reconfigure::Server<mbzirc_husky::symbolicMapConfig> dynamicServer;
	dynamic_reconfigure::Server<mbzirc_husky::symbolicMapConfig>::CallbackType fc = boost::bind(&clusterCallback, _1, _2);
	dynamicServer.setCallback(fc);


	waypoints.clear();
	robotDelivery.clear();
	dronePile.clear();
	droneDelivery.clear();
	redBricks.clear();
	blueBricks.clear();
	greenBricks.clear();
	orangeBricks.clear();
	loadWaypoints();
	ros::spin();

}

