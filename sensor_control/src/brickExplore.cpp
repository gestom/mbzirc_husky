#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <mbzirc_husky/brickExploreAction.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <mbzirc_husky/brick_pileConfig.h>
#include <vector>
#include <opencv2/core/types.hpp>
#include <mbzirc_husky_msgs/Float64.h>
#include <mbzirc_husky_msgs/brickGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
//For labelling the brick pile
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <sstream>
#include <stdlib.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#define PI 3.14159265358979

typedef actionlib::SimpleActionServer<mbzirc_husky::brickExploreAction> Server;
Server *server;

ros::ServiceClient prepareClient;
ros::Subscriber scan_sub;
ros::Publisher point_pub;
ros::Publisher point_two_pub;
ros::Publisher point_of_inter_pub;
ros::Publisher debugVisualiser;

typedef enum{
	IDLE = 0,
	EXPLORINGBRICKS,
    MOVINGTOBRICKS,
    BRICKAPPROACH,
    EXPLORINGSTACKSITE,
    MOVINGTOSTACKSITE,
    STACKAPPROACH,
    FINAL,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL
}ESprayState;
ESprayState state = IDLE;
ros::NodeHandle* pn;

float tolerance = 0.12;
float angleTolerance = 0.2;
float distanceTolerance = 0.4;
float distance = 0.3;
int minPoints = 0;
int maxEvalu = 360;
float spdLimit = 0.3;
float realConst = 1;
float dispConst = 3; 
float fwSpeed = 0.1;

int misdetections = 0;

//brickStackLocation in map frame
bool brickStackLocationKnown = false;
float brickStackRedX = 0.0f;
float brickStackRedY = 0.0f;
float brickStackOrangeX = 0.0f;
float brickStackOrangeY = 0.0f;

ros::Subscriber schedulerSub;
int redBricksRequired = 0;
int greenBricksRequired = 0;
int blueBricksRequired = 0;
int orangeBricksRequired = 0;

ros::Subscriber locationDebug;


actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* movebaseAC;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr pcl_msg (new PointCloud);
PointCloud::Ptr pcl_two_line_msg (new PointCloud);
PointCloud::Ptr pcl_of_interest_msg (new PointCloud);

void callback(mbzirc_husky::brick_pileConfig &config, uint32_t level) {

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
}

bool isTerminal(ESprayState state)
{
	if(state == EXPLORINGBRICKS) return false;
	if(state == MOVINGTOBRICKS) return false;
	if(state == BRICKAPPROACH) return false;
    if(state == EXPLORINGSTACKSITE) return false;
	if(state == MOVINGTOSTACKSITE) return false;
    if(state == STACKAPPROACH) return false;
    if(state == FINAL) return true;
	return true;
}

void precise(float *ai,float *bi,float *x,float *y,int num_ranges)
{
	float a = *ai;
	float b = *bi;
	float sxx,sxy,sx,sy;
	int n = 0;
	sxx=sxy=sx=sy=0;
	for (int j = 0; j <= num_ranges; j++){
		if (fabs(a*x[j]-y[j]+b)<tolerance){
			sx += x[j];
			sy += y[j];
			sxx += x[j]*x[j];
			sxy += x[j]*y[j];
			n++;
		}
	}
	if ((n*sxx-sx*sx) != 0 && n > 0){
		a = (n*sxy-sx*sy)/(n*sxx-sx*sx);
		b = (sy-a*sx)/n;
		*ai=a;
		*bi=b;
	}
}

double dist(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}


void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{

	size_t num_ranges = scan_msg->ranges.size();
	float x[num_ranges];
	float y[num_ranges];
	bool m[num_ranges];
	float angle;
	misdetections++;
	std::vector<std::vector<cv::Point>> ransacLines;
	ransacLines.clear();
	for (int i = 0; i <= num_ranges; i++){
		angle = scan_msg->angle_min+i*scan_msg->angle_increment;
		if(angle < 0 && angle > -(3.14/2) ){
			x[i] = scan_msg->ranges[i]*cos(angle);
			y[i] = scan_msg->ranges[i]*sin(angle);
			m[i] = true;
		}
	}
	int eval = 0;
	int max_iterations = 1000;
	int numHypotheses = 10;
	float a,b;
	float maxA[numHypotheses];
	float maxB[numHypotheses];
	int maxEval[numHypotheses];
	for (int h = 0; h <= numHypotheses; h++){
		maxEval[h] = 0;
		for (int i = 0; i <= max_iterations; i++)
		{
			int aIndex = rand()%num_ranges; 
			int bIndex = rand()%num_ranges;
			while (bIndex == aIndex) bIndex = rand()%num_ranges;
			if((x[bIndex]-x[aIndex]) == 0) continue;
			a = (y[bIndex]-y[aIndex])/(x[bIndex]-x[aIndex]);	
			b = y[bIndex]-a*x[bIndex];
			eval = 0;
			for (int j = 0; j <= num_ranges; j++){
				if (fabs(a*x[j]-y[j]+b)<tolerance && m[j]) eval++;
			}
			if (maxEval[h] < eval){
				maxEval[h]=eval;
				maxA[h]=a;
				maxB[h]=b;
			}
		}
		for (int j = 0; j <= num_ranges; j++){
			if (fabs(maxA[h]*x[j]-y[j]+maxB[h])<tolerance && m[j]) m[j] = false;
		}
	}
	eval = 0;
	int max_idx;
	for (int h1 = 0; h1 <= numHypotheses; h1++){
		precise(&maxA[h1],&maxB[h1],x,y,num_ranges);	
		if(maxEval[h1]>eval && maxEval[h1] > minPoints) {
			eval = maxEval[h1];
			max_idx = h1; 
		}	
	}

	/*
	if( STATE == RANDOM){
		//Wall detection
		pcl_msg->header.frame_id = "velodyne";
		pcl_msg->height = 1;
		pcl_msg->points.clear();
		pcl_msg->width = 0;

		std::vector<float> distan;
		distan.clear();
		int first_idx[numHypotheses];
		int last_idx[numHypotheses];
		bool first[numHypotheses];

		for(int i = 0; i < numHypotheses; i++){

			first[i]=true;
			last_idx[i] = 0;
			std::vector<cv::Point> line;
			line.clear();
			cv::Point tmp;
			double tmpx;
			double tmpy;
			double d;
			for (int j = 0; j <= num_ranges; j++)
			{
				if (fabs(maxA[i]*x[j]-y[j]+maxB[i])<tolerance && maxEval[i] > minPoints) {
					pcl_msg->points.push_back (pcl::PointXYZ(x[j], y[j], 0.1));
					pcl_msg->width++;
					tmp.x = x[j];
					tmp.y = y[j];
					line.push_back(tmp);

					//Look for first and last point in the detected line 	
					if(first[i]){
						first_idx[i] = j;
						first[i] = false;					
					}
					if(last_idx[i] < j){
						last_idx[i] = j;	
					}
					//d = dist(x[j],y[j],tmpx,tmpy);			
					//distan.push_back(d);	
				}

			}	
			ransacLines.push_back(line);
		}


		//Printing length between points 
		//for(int i =0; i < distan.size(); i++) std::cout << " " << distan[i];
		//std::cout << std::endl;	

		//Get the length of the lines 
		double line_len;
		double dist_to_poi = 0;
		double line_len_poi = 0;
		int poi_f_idx;
		int poi_l_idx;
		int poi_hypo;
		for(int i=0;i<ransacLines.size();i++) {
			if(first_idx[i] > 0 && last_idx[i] >= 1){
				line_len = dist(x[first_idx[i]],y[first_idx[i]],x[last_idx[i]],y[last_idx[i]]);
				//float line_le = dist((*)ransacLines[i][ransacLines[i].begin()].x,*ransacLines[i][ransacLines.begin()].y,*ransacLines[i][ransacLines.end()].x,*ransacLines[i][ransacLines.end()].y);
				//std::cout << "Line length " << i << " " << line_len << std::endl;
				//std::cout << "Line length r" << i << " " << line_le << std::endl;

				if(line_len > 1 && line_len < 8 && line_len_poi < line_len){
					line_len_poi = line_len;
					poi_f_idx = first_idx[i];
					poi_l_idx = last_idx[i];
					poi_hypo = i;
					displacement = (maxB[b1]+maxB[b2])/2.0;
					realAngle = (maxA[b1]+maxA[b2])/2.0;
				}
			}
		}	

		pcl_of_interest_msg->header.frame_id = "velodyne";
		pcl_of_interest_msg->height = 1;
		pcl_of_interest_msg->points.clear();
		pcl_of_interest_msg->width = 0;

		if(line_len_poi > 1 && line_len_poi < 8) {

			//std::cout<< "Point of interest X"  << (x[poi_l_idx]+x[poi_f_idx])/2 << " Y " <<  (y[poi_l_idx]+y[poi_f_idx])/2 << std::endl;	
			pcl_of_interest_msg->points.push_back (pcl::PointXYZ((x[poi_f_idx]+x[poi_l_idx])/2, (y[poi_l_idx]+y[poi_f_idx])/2 , 0));
			pcl_of_interest_msg->width++;

			dist_to_poi = dist((x[poi_l_idx]+x[poi_f_idx])/2,(y[poi_l_idx]+y[poi_f_idx])/2, 0,0);
			//std::cout << "Distance to the point of interest: " << dist_to_poi << std::endl;

			//For debug only
			for (int j = 0; j <= num_ranges; j++){
				if (fabs(maxA[poi_hypo]*x[j]-y[j]+maxB[poi_hypo])<tolerance){
					pcl_msg->points.push_back (pcl::PointXYZ(x[j], y[j], 0.1));
					pcl_msg->width++;

				}
			}

		}


		pcl_conversions::toPCL(ros::Time::now(), pcl_msg->header.stamp);
		point_pub.publish (pcl_msg);
		pcl_conversions::toPCL(ros::Time::now(), pcl_of_interest_msg->header.stamp);
		point_of_inter_pub.publish (pcl_of_interest_msg);
	}

	*/
		
	//Finding the two best used for pile detection from front 
	//if(STATUS == EXPLORINGBRICKSITE){ 	
	int b1 = -1;
	int b2 = -1;
	eval = 0; 
	float realDist,realAngle,displacement;
	for (int h1 = 0; h1 <= numHypotheses; h1++){
		for (int h2 = h1+1; h2 <= numHypotheses; h2++){
			if (fabs(maxA[h1]-maxA[h2]) < angleTolerance && maxEval[h1] > minPoints && maxEval[h2] > minPoints ){
				realAngle = (maxA[h1]+maxA[h2])/2.0;
				realDist = fabs(maxB[h1]-maxB[h2])*cos(atan(realAngle));
							//fprintf(stdout,"Brick hypothesis: %i %i %f %f %i %i\n",h1,h2,realDist,fabs(maxA[h1]-maxA[h2]),maxEval[h1],maxEval[h2]);
				//if (fabs(realDist-distance)<distanceTolerance){
					if (maxEval[h1]+maxEval[h2] > eval){
						eval = maxEval[h1] + maxEval[h2];
						b1 = h1;
						b2 = h2;
					}
				//}
			}
		}
	}


	pcl_of_interest_msg->header.frame_id = "velodyne";
	pcl_of_interest_msg->height = 1;
	pcl_of_interest_msg->points.clear();
	pcl_of_interest_msg->width = 0;
	
	pcl_msg->header.frame_id = "velodyne";
	pcl_msg->height = 1;
	pcl_msg->points.clear();
	pcl_msg->width = 0;


	pcl_two_line_msg->header.frame_id = "velodyne";
	pcl_two_line_msg->height = 1;
	pcl_two_line_msg->points.clear();
	pcl_two_line_msg->width = 0;
	
	float red_side_x;
	float red_side_y;
	float ax,bx;
	float xx,yy,xxx,yyy,xo,yo; 
	double x4; 
	double y4; 
	//std::cout << b1 << std::endl;

	if (b1 >= 0 && b2 >=0){

		bool f = true;
		bool ff = true;
		bool s = false;
		for ( int j = 0; j <= num_ranges; j++){

			if (fabs(maxA[b1]*x[j]-y[j]+maxB[b1])<tolerance)  {
				pcl_two_line_msg->points.push_back (pcl::PointXYZ(x[j], y[j], 0.15));
				pcl_two_line_msg->width++;
				if(f){
					red_side_x = x[j]; 
					red_side_y = y[j];
					f = false;
				}

			}else if (fabs(maxA[b2]*x[j]-y[j]+maxB[b2])<tolerance){
				pcl_msg->points.push_back (pcl::PointXYZ(x[j], y[j], 0.1));
				pcl_msg->width++;
				if(s){
					xxx = x[j]; 
					yyy = y[j];
				}	
				if(ff){
					xx = x[j]; 
					yy = y[j];
					ff = false;
					s = true;
				}
					xo = x[j];
					yo = y[j];

			}
			
					displacement = (maxB[b1]+maxB[b2])/2.0;
					realAngle = (maxA[b1]+maxA[b2])/2.0;
			//fprintf(stdout,"Ramp found: %i %i %f %f %f %i %i\n",b1,b2,displacement,realAngle,fabs(maxA[b1]-maxA[b2]),maxEval[b1],maxEval[b2]);

		}

		//Compute edge of pile
		//Normalized of line 
		double dx = xx - xxx;
		double dy = yy - yyy;
		double magnitude = sqrt(dx*dx + dy*dy);
		dx = dx / magnitude;
		dy = dy / magnitude;
		double lambda = (dx * (red_side_x - xxx)) + (dy * (red_side_y - yyy));
		//brickStackRedX = (dx * lambda) + xxx;
		//brickStackRedY = (dy * lambda) + yyy;
		x4 = (dx * lambda) + xxx;
		y4 = (dy * lambda) + yyy;
		//brickStackLocationKnown = true;
		
		//brickStackOrangeX = xo;
		//brickStackOrangeY = yo;

		pcl_of_interest_msg->points.push_back (pcl::PointXYZ(x4,y4, 0));
		pcl_of_interest_msg->points.push_back (pcl::PointXYZ(xo,yo, 0));
		pcl_of_interest_msg->width++;
		pcl_of_interest_msg->width++;
	}	
	pcl_conversions::toPCL(ros::Time::now(), pcl_msg->header.stamp);
	point_pub.publish (pcl_msg);
	pcl_conversions::toPCL(ros::Time::now(), pcl_two_line_msg->header.stamp);
	point_two_pub.publish (pcl_two_line_msg);
	pcl_conversions::toPCL(ros::Time::now(), pcl_of_interest_msg->header.stamp);
	point_of_inter_pub.publish (pcl_of_interest_msg);
	//	}*/
}


void schedulerCallback(const mbzirc_husky_msgs::brickGoal::ConstPtr& msg)
{
    redBricksRequired = 0;
    greenBricksRequired = 0;
    blueBricksRequired = 0;
    orangeBricksRequired = 0;

    for(int i = 0; i < msg->brickType.size(); i++)
    {
        if(msg->brickType[i] == 0)
            redBricksRequired++;
        else if(msg->brickType[i] == 1)
            greenBricksRequired++;
        else if(msg->brickType[i] == 2)
            blueBricksRequired++;
        else if(msg->brickType[i] == 3)
            orangeBricksRequired++;
    }
}

void approachBricks()
{
    schedulerSub = pn->subscribe("/brickScheduler/goals", 1, schedulerCallback);
    state = MOVINGTOBRICKS;
}

void positionArm()
{
    ROS_INFO("MOVING ARM INTO POSITION WHILE EXPLORING");
    mbzirc_husky_msgs::Float64 srv;
    srv.request.data = 0;
    prepareClient.call(srv);
}

void moveToBrick(int brick)
{
    //brick should be in range 1-4 representing second red brick to blue brick
    if(brick < 1 || brick > 4)
        ROS_INFO("BRICK ERROR out of bounds");
    
    //get front/back normals of brick stack
    float dx = brickStackOrangeX - brickStackRedX;
    float dy = brickStackOrangeY - brickStackRedY;
    if (sqrt(dx*dx+dy*dy) < 1.0) brickStackLocationKnown = false;
    printf("BRICK %i: %f %f %f %f\n",brick,brickStackOrangeX,brickStackRedX,brickStackOrangeY,brickStackRedY);
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0,0,atan2(dy,dx));
    float orientationZ = quat_tf.z();
    float orientationW = quat_tf.w();

    //offset for bricks on parallel line
    float offset = 1.0f;
    if(brick >= 1)
        offset += 0.4;
    if(brick >= 2)
        offset += 0.4 + .15 + 0.5 + 0.3;
    if(brick >= 3)
        offset += 0.7;
    if(brick >= 4)
        offset += 0.3 + 0.5 + 0.6;
    offset += 0.68;
    offset += 0.35;

	ROS_INFO("Brick forward offset %f", offset);

    float frontNormalX = -dy;
    float frontNormalY = dx;

    //normalise normals
    float magnitude = pow(pow(frontNormalX, 2) + pow(frontNormalY,2), 0.5);
    frontNormalX /= magnitude;
    frontNormalY /= magnitude;
    float gradientX = dx / magnitude;
    float gradientY = dy / magnitude;

    float wayPointX = -offset;
    float wayPointY = 0.45f;
    float stackDepth = 0.4; //half the depth ie 1.5 blocks plus 10cm gap
    const float originX = brickStackRedX + 0;//(frontNormalX * stackDepth);
    const float originY = brickStackRedY + 0;//(frontNormalY * stackDepth);
    //add the y
    printf("AA:A: %f %f %f\n",originX,frontNormalX,wayPointY);
    float mapWPX = originX + (frontNormalX * wayPointY);
    float mapWPY = originY + (frontNormalY * wayPointY);
    //add the x
    mapWPX -= (gradientX * (wayPointX)); 
    mapWPY -= (gradientY * (wayPointX));
   
	ROS_INFO("MOVING TO POS %f %f", mapWPX, mapWPY);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;

    marker.scale.x = 0.4;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;

    marker.pose.position.x = mapWPX;
    marker.pose.position.y = mapWPY;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = orientationZ;
    marker.pose.orientation.w = orientationW;
    
    debugVisualiser.publish(marker);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = mapWPX;
    goal.target_pose.pose.position.y = mapWPY;

    //goal orientation
    goal.target_pose.pose.orientation.z = orientationZ;
    goal.target_pose.pose.orientation.w = orientationW;

    ROS_INFO("Moving to brick position");
    movebaseAC->sendGoal(goal);
    movebaseAC->waitForResult();
    //ROS_INFO(movebaseAC.getState());
    if(movebaseAC->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	    ROS_INFO("Approached, moving to brick");
    else
	    ROS_INFO("FAILED on first approach, continueing");
    ROS_INFO("Approached, done");
    state=FINAL;
}

void moveToApproachWP()
{
    //get front/back normals of brick stack
    float dx = brickStackOrangeX - brickStackRedX;
    float dy = brickStackOrangeY - brickStackRedY;
    if (sqrt(dx*dx+dy*dy) < 1.0) brickStackLocationKnown = false;
    printf("START: %f %f %f %f\n",brickStackOrangeX,brickStackRedX,brickStackOrangeY,brickStackRedY);
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0,0,atan2(dy,dx));

    float finalOrientationTheta = (PI) - atan2(dy, dx);

    //float finalOrientationZ = sin(finalOrientationTheta / 2);
    //float finalOrientationW = cos(finalOrientationTheta / 2);
    float finalOrientationZ = quat_tf.z();
    float finalOrientationW = quat_tf.w();

    float frontNormalX = -dy;
    float frontNormalY = dx;

    //normalise normals
    float magnitude = pow(pow(frontNormalX, 2) + pow(frontNormalY,2), 0.5);
    frontNormalX /= magnitude;
    frontNormalY /= magnitude;
    float gradientX = dx / magnitude;
    float gradientY = dy / magnitude;

    //now we can calculate waypoint position in stack frame
    //the following x,y are the approach path waypoints rel to stack
    //where 0, 0 equals the red side of the stack, closest right corner
    //and +ve x moves to the right of the stack
    //and if facing the front of the stack +ve y steps back
    //basically just as in the spec book
    //all in map frame
    float wayPointX = 2.f;
    float wayPointY = 0.45f;
    float stackDepth = 0.4; //half the depth ie 1.5 blocks plus 10cm gap
    const float originX = brickStackRedX + 0;//(frontNormalX * stackDepth);
    const float originY = brickStackRedY + 0;//(frontNormalY * stackDepth);
    //add the y
    printf("AA:A: %f %f %f\n",originX,frontNormalX,wayPointY);
    float mapWPX = originX + (frontNormalX * wayPointY);
    float mapWPY = originY + (frontNormalY * wayPointY);
    //add the x
    mapWPX -= (gradientX * (wayPointX+0.2));
    mapWPY -= (gradientY * (wayPointX+0.2));
    //orientation
    float orientationTheta = (PI) - atan2((wayPointY - originY), (wayPointX - originX));
    //float orientationZ = sin(orientationTheta / 2);
    //float orientationW = cos(orientationTheta / 2);
    float orientationZ = quat_tf.z();
    float orientationW = quat_tf.w();

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;

    marker.scale.x = 0.4;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;

    marker.pose.position.x = mapWPX;
    marker.pose.position.y = mapWPY;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = orientationZ;
    marker.pose.orientation.w = orientationW;
    
    debugVisualiser.publish(marker);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = mapWPX;
    goal.target_pose.pose.position.y = mapWPY;

    //goal orientation
    goal.target_pose.pose.orientation.z = orientationZ;
    goal.target_pose.pose.orientation.w = orientationW;

    ROS_INFO("Moving to approach position");
    movebaseAC->sendGoal(goal);
    movebaseAC->waitForResult();
    //ROS_INFO(movebaseAC.getState());
    if(movebaseAC->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	    ROS_INFO("Approached, moving to brick");
    else
	    ROS_INFO("FAILED on first approach, continueing");

    //fire off command to get arm ready
    positionArm();
	printf("ALALAL: %f %f %f %f %f %f %f\n",mapWPX,originX,frontNormalX,wayPointY,gradientX,gradientY,magnitude);

	mapWPX = originX + (frontNormalX * wayPointY) + (gradientX * 1);
	mapWPY = originY + (frontNormalY * wayPointY) + (gradientY * 1);

    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::ARROW;

    marker.pose.position.x = mapWPX;
    marker.pose.position.y = mapWPY;
    marker.pose.position.z = 0.3;
    marker.pose.orientation.z = finalOrientationZ;
    marker.pose.orientation.w = finalOrientationW;
    
    debugVisualiser.publish(marker);

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = mapWPX;
    goal.target_pose.pose.position.y = mapWPY;
    goal.target_pose.pose.orientation.z = finalOrientationZ;
    goal.target_pose.pose.orientation.w = finalOrientationW;
	ROS_INFO("%f", mapWPX);
	ROS_INFO("%f", mapWPY);

    ROS_INFO("Moving to brick position");
    movebaseAC->sendGoal(goal);
    movebaseAC->waitForResult();
    if(movebaseAC->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	    ROS_INFO("Approached, moving to brick");
    else
	    ROS_INFO("FAILED on first approach, continueing");

    //ROS_INFO(movebaseAC.getState());
    ROS_INFO("Approached, done");
    state = FINAL;
}

void locationDebugCallback(const std_msgs::String::ConstPtr& msg)
{
	if(brickStackLocationKnown)
		return;
	ROS_INFO("RECEIVED POS");
	char* ch;
	ch = strtok(strdup(msg->data.c_str()), " ");
	int varIdx = 0;
	while(ch != NULL)
	{
		if(varIdx == 0)
			brickStackRedX = atof(ch);
		else if(varIdx == 1)
			brickStackRedY = atof(ch);
		else if(varIdx == 2)
			brickStackOrangeX = atof(ch);
		else if(varIdx == 3)
			brickStackOrangeY = atof(ch);
		varIdx++;
		ch = strtok(NULL, " ");
	}
	ROS_INFO("POSITION STORED");
	brickStackLocationKnown = true;
}

void moveToBricks(int brick)
{
	if(brickStackLocationKnown)
	{
        if(brick == 0)
        {
		    ROS_INFO("Moving to first brick");
    		moveToApproachWP();
        }
        else
        {
            moveToBrick(brick);
        }
	}
	else
	{
        ROS_INFO("Approach waiting for ransac position...");
        usleep(1000000);		
	}
}

void actionServerCallback(const mbzirc_husky::brickExploreGoalConstPtr &goal, Server* as)
{
    mbzirc_husky::brickExploreResult result;
    
    //goal->goal == 0 brick pickup, goal->goal == 1 stack site
    //goal->brick == 0-4 for the current brick to handle
	ROS_INFO("Goal received %i %i", goal->goal, goal->brick);

    if(goal->goal == 1)
    {
        //brick pick up
        if(goal->brick == 0)
        {
            state = EXPLORINGBRICKS;
        }
        else
        {
            state = MOVINGTOBRICKS;    
        }
    }
    else if(goal->goal == 2)
    {
        //stack site
        state = EXPLORINGSTACKSITE;
    }

    while (isTerminal(state) == false && ros::ok()){
        if(state == EXPLORINGBRICKS)
        {
            //begin lidar search for bricks
            usleep(2000000);
            approachBricks();
        }
        else if(state == MOVINGTOBRICKS)
        {
            moveToBricks(goal->brick);
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
	ros::init(argc, argv, "brickExplore");
	ros::NodeHandle n;
   	pn = &n;
    movebaseAC = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
    debugVisualiser = n.advertise<visualization_msgs::Marker>("/brickExplore/debug", 1);
	// Dynamic reconfiguration server
	dynamic_reconfigure::Server<mbzirc_husky::brick_pileConfig> dynServer;
  	dynamic_reconfigure::Server<mbzirc_husky::brick_pileConfig>::CallbackType f = boost::bind(&callback, _1, _2);
  	dynServer.setCallback(f);
	prepareClient = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/prepare_gripping");
    scan_sub = n.subscribe("/scan",100, scanCallback);	
	point_pub = n.advertise<sensor_msgs::PointCloud2>("ransac/correct_one_line",10);
	point_two_pub = n.advertise<sensor_msgs::PointCloud2>("ransac/correct_two_lines",10);
	locationDebug = n.subscribe("/locationDebug", 1, locationDebugCallback);
	point_of_inter_pub = n.advertise<sensor_msgs::PointCloud2>("ransac/poi",10);
	server = new Server(n, "brickExploreServer", boost::bind(&actionServerCallback, _1, server), false);
	server->start();
	while (ros::ok()){
		if (server->isPreemptRequested() && state != IDLE) state = PREEMPTED;
		if (state == STOPPING){
			state = IDLE;
		} 
		ros::spinOnce();
		usleep(1000);
	}
    delete server;
    delete movebaseAC;
}
