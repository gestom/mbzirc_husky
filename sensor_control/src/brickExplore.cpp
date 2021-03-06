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
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <mbzirc_husky_msgs/Float64.h>
#include <mbzirc_husky_msgs/getPoi.h>
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
#include <mbzirc_husky_msgs/wallPatternDetect.h>
#include <std_srvs/Trigger.h>
#include <mbzirc_husky_msgs/brick_pile_trigger.h>
#define PI 3.14159265358979

typedef actionlib::SimpleActionServer<mbzirc_husky::brickExploreAction> Server;
Server *server;

ros::ServiceClient prepareClient;
ros::ServiceClient symbolicClient;
ros::ServiceClient wprosbagClient;
ros::ServiceClient wallSearchClient;
ros::ServiceClient brickPileDetectorClient;
ros::ServiceClient wallPatternInvestigatorClient;
ros::Subscriber scan_sub;
ros::Subscriber podvod_sub;
ros::Publisher ransac_pub;
ros::Publisher point_pub;
ros::Publisher point_two_pub;
ros::Publisher point_of_inter_pub;
ros::Publisher debugVisualiser;
ros::Publisher bricksVisualiser;

typedef enum{
	IDLE = 0,
	EXPLORINGBRICKS,
    INVESTIGATEWP,
    INVESTIGATEBRICKS,
    MOVINGTOBRICKS,
    PRECISEBRICKFIND,
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

float tolerance = 0.03;
float angleTolerance = 0.05;
float distanceTolerance = 0.4;
float distance = 0.3;
int minPoints = 0;
int maxEvalu = 360;
float spdLimit = 0.3;
float realConst = 1;
float dispConst = 3; 
float fwSpeed = 0.1;
int covarianceBricks = 500;
int covariancePattern = 1500;

bool podvod = false;

int misdetections = 0;

//brickStackLocation in map frame
//bool brickStackLocationKnown = false;
//bool wallPatternLocationKnown = false;
bool brickStackLocationKnown = false;
bool wallPatternLocationKnown = false;
float brickStackLocationX = 0.0f;
float brickStackLocationY = 0.0f;
float wallPatternLocationX = 0.0f;
float wallPatternLocationY = 0.0f;
float wallPatternOrX = 0.0f;
float wallPatternOrY = 0.0f;

float investigateX = 0.0f;
float investigateY = 0.0f;

bool useRansac = false;
bool precisePositionFound = false;
float brickStackRedX = -0;
float brickStackRedY = 0;
float brickStackOrangeX = 1.866;
float brickStackOrangeY = 31.397;

ros::Subscriber schedulerSub;
int redBricksRequired = 0;
int greenBricksRequired = 0;
int blueBricksRequired = 0;
int orangeBricksRequired = 0;

int moveToBrickPosition(float x, float y, float orientationOffset, float tolerance);
int moveToMapPoint(float x, float y, float orientationZ, float orientationW, float tolerance);

ros::Subscriber preciseLocation;

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* movebaseAC;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr pcl_msg (new PointCloud);
PointCloud::Ptr pcl_two_line_msg (new PointCloud);
PointCloud::Ptr pcl_of_interest_msg (new PointCloud);

void dynamicReconfigureCallback(mbzirc_husky::brick_pileConfig &config, uint32_t level) {

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
        covarianceBricks=config.covarianceBricks;
        covariancePattern=config.covariancePattern;
}

bool isTerminal(ESprayState state)
{
	if(state == EXPLORINGBRICKS) return false;
	if(state == INVESTIGATEWP) return false;
	if(state == INVESTIGATEBRICKS) return false;
	if(state == MOVINGTOBRICKS) return false;
	if(state == PRECISEBRICKFIND) return false;
	if(state == BRICKAPPROACH) return false;
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

void podvodCallback(const std_msgs::String::ConstPtr& msg)
{	
	brickStackLocationKnown = true;
	podvod = true;
	ROS_INFO("PODVOD ACTIVATED");    
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
		else if(varIdx == 4)
		{
			wallPatternLocationKnown = true;
			wallPatternLocationX = atof(ch);
		}
		else if(varIdx == 5)
			wallPatternLocationY = atof(ch);
		else if(varIdx == 6)
			wallPatternOrX = atof(ch);
		else if(varIdx == 7)
			wallPatternOrY = atof(ch);
		varIdx++;
		ch = strtok(NULL, " ");
	}

}

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	if(state != PRECISEBRICKFIND)
		return;
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
			x[i] = scan_msg->ranges[i]*cos(angle);
			y[i] = scan_msg->ranges[i]*sin(angle);
			m[i] = true;
	}
	int eval = 0;
	int max_iterations = 1200;
	int numHypotheses = 15;
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
				if (fabs(realDist-distance)<distanceTolerance){
							//fprintf(stdout,"Brick hypothesis: %i %i %f %f %i %i\n",h1,h2,realDist,fabs(maxA[h1]-maxA[h2]),maxEval[h1],maxEval[h2]);
					if (maxEval[h1]+maxEval[h2] > eval){
						eval = maxEval[h1] + maxEval[h2];
						b1 = h1;
						b2 = h2;
					}
				}
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
	float xf,xo,yo,xs,lxf,lxs,yf,ys,lyf,lys,xx,yy,xxx,yyy; 
	double x4; 
	double y4; 

	if (b1 >= 0 && b2 >=0){

		bool f = true;
		bool ff = true;
		bool ss = false;
		bool s = false;
		for ( int j = 0; j <= num_ranges; j++){
			angle = scan_msg->angle_min+j*scan_msg->angle_increment;
		if(angle < (3.1415/10) && angle > -(3.14/2) ){ 	

			if (fabs(maxA[b1]*x[j]-y[j]+maxB[b1])<tolerance) {
				pcl_two_line_msg->points.push_back (pcl::PointXYZ(x[j], y[j], 0.15));
				pcl_two_line_msg->width++;


					if(f){
						xf = x[j]; 
						yf = y[j];
						f = false;
						s = true;
					}
					if(s){

						lxf = x[j]; 
						lyf = y[j];
					}	

			}else if (fabs(maxA[b2]*x[j]-y[j]+maxB[b2])<tolerance){
				pcl_msg->points.push_back (pcl::PointXYZ(x[j], y[j], 0.1));
				pcl_msg->width++;
					if(ff){
						xs = x[j]; 
						ys = y[j];
						ff = false;
						ss = true;
					}

					if(ss){
						lxs = x[j]; 
						lys = y[j];
					}	
				}
			

		}
		if(abs(maxB[b1]) > abs(maxB[b2])){
			red_side_x = xf;
			red_side_y = yf;
			xx = xs;
			yy = ys;
			xxx = lxs;
			yyy = lys;
		}else{
			red_side_x = xs;
			red_side_y = ys;
			xx = xf;
			yy = yf;
			xxx = lxf;
			yyy = lyf;
		}
			xo = xxx;
			yo = yyy;
			displacement = (maxB[b1]+maxB[b2])/2.0;
			realAngle = (maxA[b1]+maxA[b2])/2.0;
			//fprintf(stdout,"Ramp found: %i %i %f %f %f %i %i\n",b1,b2,displacement,realAngle,fabs(maxA[b1]-maxA[b2]),maxEval[b1],maxEval[b2]);

		//Compute edge of pile
		//Normalized of line 
		double dx = xx - xxx;
		double dy = yy - yyy;
		double magnitude = sqrt(dx*dx + dy*dy);
		dx = dx / magnitude;
		dy = dy / magnitude;
		double lambda = (dx * (red_side_x - xxx)) + (dy * (red_side_y - yyy));
		x4 = (dx * lambda) + xxx;
		y4 = (dy * lambda) + yyy;
		brickStackLocationKnown = true;

		if(dist(x4,y4,0,0) > 0.5){
		//Lookup in the past
		
		brickStackOrangeX = xo;
		brickStackOrangeY = yo;
		/*tf::TransformListener listener;
		tf::StampedTransform transform;
		listener.waitForTransform("map", "base_link",
                             ros::Time(0), ros::Duration(1));

		//try{
			listener.lookupTransform("map", "base_link",
				ros::Time(0) ,transform);
		//}
		//catch (tf::TransformException &ex) {
		//	ROS_ERROR("%s",ex.what());
		//	ros::Duration(1.0).sleep();
			//return;
	//	}
	
		//brickStackRedX = (dx * lambda) + xxx;
		//brickStackRedY = (dy * lambda) + yyy;
		x4 = (dx * lambda) + xxx;
		y4 = (dy * lambda) + yyy;
		//brickStackLocationKnown = true;
		
		//brickStackOrangeX = xo;
		//brickStackOrangeY = yo;

		brickStackRedX	 = x4 + transform.getOrigin().y();
		brickStackRedY	 = y4 + transform.getOrigin().y();
		brickStackOrangeX = x4 + transform.getOrigin().y();
		brickStackOrangeX  = y4 + transform.getOrigin().y();

		std::cout<< " map " << brickStackRedX << " velodyne " << x4 << std::endl;*/
		pcl_of_interest_msg->points.push_back (pcl::PointXYZ(x4,y4, 0));
		pcl_of_interest_msg->width++;
		pcl_of_interest_msg->points.push_back (pcl::PointXYZ(xo,yo, 0));
		pcl_of_interest_msg->width++;

		
		}
		//std::cout << pcl_of_interest_msg->width << std::endl;
		pcl_conversions::toPCL(ros::Time::now(), pcl_of_interest_msg->header.stamp);
		point_of_inter_pub.publish (pcl_of_interest_msg);
		pcl_of_interest_msg->points.clear();
		pcl_of_interest_msg->width = 0;
		}
	}	
	pcl_conversions::toPCL(ros::Time::now(), pcl_msg->header.stamp);
	point_pub.publish (pcl_msg);
	pcl_conversions::toPCL(ros::Time::now(), pcl_two_line_msg->header.stamp);
	point_two_pub.publish (pcl_two_line_msg);
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

void positionArm()
{
    ROS_INFO("MOVING ARM INTO POSITION WHILE EXPLORING");
    mbzirc_husky_msgs::Float64 srv;
    srv.request.data = 0;
    prepareClient.call(srv);
}

void preciseLocationCallback(const std_msgs::String::ConstPtr& msg)
{
	if(state != PRECISEBRICKFIND)
		return;
    if(precisePositionFound)
        return; 
	
    ROS_INFO("RECEIVED RANSAC POS");
	
    brickStackLocationKnown = true;
    precisePositionFound = true;
	
    char* ch;
	ch = strtok(strdup(msg->data.c_str()), " ");
	int varIdx = 0;
    int RX, RY, OX, OY;

	while(ch != NULL)
	{
		if(varIdx == 0)
			RX = atof(ch);
		else if(varIdx == 1)
			RY = atof(ch);
		else if(varIdx == 2)
			OX = atof(ch);
		else if(varIdx == 3)
			OY = atof(ch);
		varIdx++;
		ch = strtok(NULL, " ");
	}

    tf::TransformListener listener;
    try
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.header.stamp = ros::Time(0);
        pose.pose.position.x = OX;
        pose.pose.position.y = OY;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        listener.waitForTransform("/base_link", "map", ros::Time(0), ros::Duration(1.0));
        geometry_msgs::PoseStamped newPose;
        listener.transformPose("/map", pose, newPose);
        brickStackOrangeX = newPose.pose.position.x;
        brickStackOrangeY = newPose.pose.position.y;
        pose.pose.position.x = RX; 
        pose.pose.position.y = RY; 
        listener.transformPose("/map", pose, newPose);
        brickStackRedX = newPose.pose.position.x;
        brickStackRedY = newPose.pose.position.y;
        state = BRICKAPPROACH;
    }
    catch (tf::TransformException &ex)
    {
        ROS_INFO("Error looking up transform %s", ex.what());
        precisePositionFound = false;
        return;
    }
}

void explore()
{
    mbzirc_husky_msgs::getPoi srv;
    srv.request.type = 0;
    if(symbolicClient.call(srv))
    {
        ROS_INFO("Moving to point");

        ROS_INFO("Zdeneks search for bricks");
        mbzirc_husky_msgs::brick_pile_trigger bpd;
        bpd.request.activate = true;
        if(!brickPileDetectorClient.call(bpd))
            ROS_INFO("Brick pile detector failed to set true");

        moveToMapPoint(srv.response.x[0], srv.response.y[0], 0, 1, 1.5);

        ROS_INFO("Searching for bricks");
        mbzirc_husky_msgs::brick_pile_trigger bpdb;
        bpdb.request.activate = false;
        if(!brickPileDetectorClient.call(bpdb))
            ROS_INFO("Brick pile detector failed to set false");

        ROS_INFO("Sending velo points");
        std_srvs::Trigger srv;
        wprosbagClient.call(srv);

        ROS_INFO("Calling wall pattern detect");
          mbzirc_husky_msgs::wallPatternDetect m;
          m.request.activate = true;
          m.request.rotate_arm = true;
          wallSearchClient.call(m);

        ROS_INFO("Finished calling wp search");
        usleep(2000000);

        /*float reqCovar = 0.5;
        mbzirc_husky_msgs::getPoi srv;
        srv.request.type = 4;
        if(symbolicClient.call(srv))
        {
            if(srv.response.covariance[0] > reqCovar)
            {
                state = INVESTIGATEBRICKS;
            }
        }

        mbzirc_husky_msgs::getPoi srv;
        srv.request.type = 5;
        if(symbolicClient.call(srv))
        {
            if(srv.response.covariance[0] > reqCovar)
            {
                state = INVESTIGATEBRICKS;
            }
        }

        mbzirc_husky_msgs::getPoi srv;
        srv.request.type = 6;
        if(symbolicClient.call(srv))
        {
            if(srv.response.covariance[0] > reqCovar)
            {
                state = INVESTIGATEBRICKS;
            }
        }

        mbzirc_husky_msgs::getPoi srv;
        srv.request.type = 7;
        if(symbolicClient.call(srv))
        {
            if(srv.response.covariance[0] > reqCovar)
            {
                state = INVESTIGATEBRICKS;
            }
        }*/
    }
    else
    {
        ROS_INFO("Service failed");
    }
}

void moveToBrickPile()
{
	//one is along, two is perp
    ROS_INFO("Approaching bricks");
    moveToBrickPosition(4, 3., -0.4, 1.5);
    ROS_INFO("Closer approach to bricks");
    moveToBrickPosition(2.5, 1.5, -0.0, 1);
    
	usleep(10000000);

    //yeah this is copied from online, but it only needs to trigger ransac reset
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world ";
    msg.data = ss.str();
    ransac_pub.publish(msg);

    usleep(3500000);

    //wait for ransac callback
    precisePositionFound = false;
    useRansac = true;
    state = PRECISEBRICKFIND;
}

void preciseBrickFind()
{
    //the ransac callback should trigger the change to the next state
    ROS_INFO("Waiting for ransac...");
    usleep(2000000);
}

int moveToBrickPosition(float x, float y, float orientationOffset, float tolerance)
{ 
    float dx = brickStackOrangeX - brickStackRedX;
    float dy = brickStackOrangeY - brickStackRedY;
    if (sqrt(dx*dx+dy*dy) < 1.0)
    {
	    ROS_INFO("ERROR WITH SQRT");
        brickStackLocationKnown = false;
        return -1;
    }

    float theta = atan2(dy, dx);
    float mapWPX = x*cos(theta) + y*sin(theta) + brickStackRedX;
    float mapWPY = -x*sin(theta) + y*cos(theta) + brickStackRedY;

    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, theta); // orientationOffset + PI);
    printf("POS: %f %f %f %f", brickStackRedX, brickStackRedY, brickStackOrangeX, brickStackOrangeY);
    printf("WWW: %f %f %f\n" ,dx,dy,theta);
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
    marker.pose.position.z = 0.1;
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
    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = orientationZ;
    goal.target_pose.pose.orientation.w = orientationW;

    movebaseAC->sendGoal(goal);
    
	while(ros::ok())
	{	
		ROS_INFO("Polling position");
		usleep(500000);
		
		tf::TransformListener listener;
		try
		{
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = "base_link";
			pose.header.stamp = ros::Time(0);
			pose.pose.position.x = 0;
			pose.pose.position.y = 0;
			pose.pose.position.z = 0;
			pose.pose.orientation.x = 0;
			pose.pose.orientation.y = 0;
			pose.pose.orientation.z = 0;
			pose.pose.orientation.w = 1;
			listener.waitForTransform("/base_link", "map", ros::Time(0), ros::Duration(1.0));
			geometry_msgs::PoseStamped newPose;
			listener.transformPose("/map", pose, newPose);

			float dx = newPose.pose.position.x - mapWPX;
			float dy = newPose.pose.position.y - mapWPY;

			ROS_INFO("Distance to target: %f", sqrt(dx*dx+dy*dy));

			if (sqrt(dx*dx+dy*dy) < tolerance)
			{
				return 0;
			}
		}
		catch (tf::TransformException &ex)
		{
			ROS_INFO("Error looking up transform %s", ex.what());
			return -1;
		}
	}	
}

void investigateWallPattern(float approachAngle)
{
	mbzirc_husky_msgs::getPoi srv;
	srv.request.type = 3;
	if(symbolicClient.call(srv))
	{
		if(srv.response.covariance[0] > covariancePattern)
        {
            float centreX = srv.response.x[0];
            float centreY = srv.response.y[0];

            float wallPatternRadius = 3.0;

            tf::TransformListener listener;
            try
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "base_link";
                pose.header.stamp = ros::Time(0);
                pose.pose.position.x = 0;
                pose.pose.position.y = 0;
                pose.pose.position.z = 0;
                pose.pose.orientation.x = 0;
                pose.pose.orientation.y = 0;
                pose.pose.orientation.z = 0;
                pose.pose.orientation.w = 1;
                listener.waitForTransform("/base_link", "map", ros::Time(0), ros::Duration(1.0));
                geometry_msgs::PoseStamped newPose;
                listener.transformPose("/map", pose, newPose);

                float vecX = centreX - newPose.pose.position.x;
                float vecY = centreY - newPose.pose.position.y;

                float mag = sqrt(vecX*vecX+vecY*vecY);
                vecX = (vecX / mag) * (mag - wallPatternRadius);
                vecY = (vecY / mag) * (mag - wallPatternRadius);

                vecX = newPose.pose.position.x + vecX;
                vecY = newPose.pose.position.y + vecY;

                moveToMapPoint(vecX, vecY, 0, 1, 0.5);

                mbzirc_husky_msgs::Float64 msg;
                msg.request.data = 0;
                wallPatternInvestigatorClient.call(msg);

                mbzirc_husky_msgs::wallPatternDetect m;
                m.request.activate = true;
                m.request.rotate_arm = false;
                wallSearchClient.call(m);

                    //call to service here
                    //
                    //check check if symbolic map contains magic number, if so change state to EXPLORE.
                    //if it doesnt, call self recursively setting angle
            }
            catch (tf::TransformException &ex)
            {
                ROS_INFO("Error looking up transform %s", ex.what());
                return;
            }               
        }
	}    
}

void investigateBricks(float approachAngle)
{
    moveToMapPoint(investigateX, investigateY, 0, 1, 10); 

    float redX = 0.0f;
    float redY = 0.0f;
    float otherX = 0.0f;
    float otherY = 0.0f;
    float otherCovar = 0.0f;

    for(int brickIdx = 0; brickIdx < 4; brickIdx++)
    {
	    if(brickIdx == 1)
		    continue;
        mbzirc_husky_msgs::getPoi srvB;
        srvB.request.type = 4 + brickIdx;

        if(symbolicClient.call(srvB))
	{
		int iterations = 5;
		if(srvB.response.x.size() < iterations)
			iterations = srvB.response.x.size();
		for(int i = iterations-1; i >= 0 ; i--)
		{
			float dx = abs(srvB.response.x[i] - investigateX);
			float dy = abs(srvB.response.y[i] - investigateY);
			float dist = sqrt(dx*dx+dy*dy);
			ROS_INFO("INVINFO %i %f %f %f", brickIdx, dx, dy, dist);

			if(dist < 10)
			{
				if(brickIdx == 0)
				{
					redX = srvB.response.x[i];
					redY = srvB.response.y[i];
				}
				else
				{
					if(srvB.response.covariance[i] >= otherCovar)
					{
						otherCovar = srvB.response.covariance[i];
						otherX = srvB.response.x[i];
						otherY = srvB.response.y[i];
					}
				}
			}
		}
	}
        else
            ROS_INFO("Symbolic map call failed!");
    }
    ROS_INFO("INVESTIGATION INFO %f %f %f %f", redX, redY, otherX, otherY);
    if(otherCovar > 0.5)
    {
        brickStackLocationKnown = true;
        brickStackRedX = redX;
        brickStackRedY = redY;
        brickStackOrangeX = otherX;
        brickStackOrangeY = otherY;
     	state = EXPLORINGBRICKS;
        ROS_INFO("FOUND BRICKS: %f %f %f %f", redX, redY, otherX, otherY);
	return;
    }
    else
    {
	state = EXPLORINGBRICKS;
    }
    ROS_INFO("CANDIDATE NOT GOOD");
}

int moveToMapPoint(float x, float y, float orientationZ, float orientationW, float tolerance)
{
	ROS_INFO("Moving to a map point %f %f", x, y);
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;

	//goal orientation
	goal.target_pose.pose.orientation.z = orientationZ;
	goal.target_pose.pose.orientation.w = orientationW;


    if(tolerance != 0)
    {
        movebaseAC->sendGoal(goal);
        while(ros::ok())
        {	
            usleep(500000);

            tf::TransformListener listener;
            try
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "base_link";
                pose.header.stamp = ros::Time(0);
                pose.pose.position.x = 0;
                pose.pose.position.y = 0;
                pose.pose.position.z = 0;
                pose.pose.orientation.x = 0;
                pose.pose.orientation.y = 0;
                pose.pose.orientation.z = 0;
                pose.pose.orientation.w = 1;
                listener.waitForTransform("/base_link", "map", ros::Time(0), ros::Duration(1.0));
                geometry_msgs::PoseStamped newPose;
                listener.transformPose("/map", pose, newPose);

                float dx = newPose.pose.position.x - x;
                float dy = newPose.pose.position.y - y;

                if (sqrt(dx*dx+dy*dy) < tolerance)
                {
                    move_base_msgs::MoveBaseGoal goal2;

                    movebaseAC->sendGoal(goal2);
                    break;
                }
            }
            catch (tf::TransformException &ex)
            {
                ROS_INFO("Error looking up transform %s", ex.what());
                return -1;
            }
        }
    }
    else
    {
        movebaseAC->sendGoalAndWait(goal);
        actionlib::SimpleClientGoalState mbState = movebaseAC->getState();
        ROS_INFO("Move base state %s", mbState.getText());

        if(mbState == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Move target achieved");
            return 0;
        }
        else
        {
            ROS_INFO("Move base timed out, current state: %s", mbState.getText().c_str());
            return -1;
        }
    }
	return 0;
}

void finalApproach()
{
	ROS_INFO("Final approach");
	if(moveToBrickPosition(1., 1.25, 0.4, 0.4) == 0)//second is perp, one is along
	{
		ROS_INFO("Approach successful");
		state = FINAL;
	}
	else
	{
		ROS_INFO("Failed final approach, going round again");
		state = MOVINGTOBRICKS;
	}
}

void actionServerCallback(const mbzirc_husky::brickExploreGoalConstPtr &goal, Server* as)
{
    mbzirc_husky::brickExploreResult result;
    
    if(goal->goal == 1)
    {
        ROS_INFO("MOVE TO BRICK PILE GOAL RECEVIED");
        state = EXPLORINGBRICKS;
    }
    else if(goal->goal == 2)
    {
        ROS_INFO("MOVE TO BRICK STACK SITE GOAL RECEIVED");
        state = MOVINGTOSTACKSITE;
    }

    while (isTerminal(state) == false && ros::ok()){
        if(state == EXPLORINGBRICKS)
        {
            if(podvod)
            {
                float dx = brickStackOrangeX - brickStackRedX;
                float dy = brickStackOrangeY - brickStackRedY;

                float theta = atan2(dy, dx);

                tf2::Quaternion quat_tf;
                quat_tf.setRPY(0, 0, theta); // orientationOffset + PI);
                printf("POS: %f %f %f %f", brickStackRedX, brickStackRedY, brickStackOrangeX, brickStackOrangeY);
                printf("WWW: %f %f %f\n" ,dx,dy,theta);

                moveToMapPoint(brickStackRedX-4, brickStackRedY-5, quat_tf.z(), quat_tf.w(), 1);
                moveToMapPoint(brickStackRedX-2.5, brickStackRedY-5.5, quat_tf.z(), quat_tf.w(), 1);
                moveToMapPoint(brickStackRedX-1.4, brickStackRedY-5, quat_tf.z(), quat_tf.w(), 0.5);
                moveToMapPoint(brickStackRedX, brickStackRedY, quat_tf.z(), quat_tf.w(), 1);

                state = FINAL;
            }
            /*if(brickStackLocationKnown && wallPatternLocationKnown)
            {
                ROS_INFO("Locations already known, moving straight to them");
                state = MOVINGTOBRICKS;
                continue;
            }


            //check for candidate wall patterns
            if(!wallPatternLocationKnown)
            {
                ROS_INFO("Looking For wall pattern");
                mbzirc_husky_msgs::getPoi srvWP;
                srvWP.request.type = 3;
                if(symbolicClient.call(srvWP))
                {
                    if(srvWP.response.covariance[0] == 666)
                    {
                        wallPatternLocationKnown = true;
                        wallPatternLocationX = (float)srvWP.response.x[0];
                        wallPatternLocationY = (float)srvWP.response.y[0];
                    }
                    else if(srvWP.response.covariance[0] > covariancePattern)
                    {
                        state = INVESTIGATEWP;
                    }
                }
                else
                    ROS_INFO("Symbolic map call failed!");
            }

            //see if any bricks were seen
            if(!brickStackLocationKnown)
            {
                ROS_INFO("Looking For bricks");

                for(int i = 0; i < 4; i++)
                {
                    mbzirc_husky_msgs::getPoi srvB;
                    srvB.request.type = 4 + i;

                    if(symbolicClient.call(srvB))
                    {
                        if(srvB.response.covariance[0] > 0.4)
                        {
                            ROS_INFO("Strong candidate to investigate");
                            investigateX = srvB.response.x[0];
                            investigateY = srvB.response.y[0];
                            state = INVESTIGATEBRICKS;
                        }
                    }
                    else
                        ROS_INFO("Symbolic map call failed!");
                }
            }*/


            //explore();
        }
        else if(state == INVESTIGATEWP)
        {
            ROS_INFO("Investigating wall pattern");
            investigateWallPattern(0.0f);
        }
        else if(state == INVESTIGATEBRICKS)
        {
            ROS_INFO("Investigating bricks");
            investigateBricks(0.0f);
        }
        else if(state == MOVINGTOBRICKS)
        {
            moveToBrickPile();
        }
        else if(state == PRECISEBRICKFIND)
        {
		    ROS_INFO("Precise brick find");
    		preciseBrickFind();
        }
        else if(state == BRICKAPPROACH)
        {
            finalApproach();
        }
        else if(state == MOVINGTOSTACKSITE)
	{
		if(podvod)
		{
			float dx = wallPatternOrX - wallPatternLocationX;
			float dy = wallPatternOrY - wallPatternLocationY;

			float theta = atan2(dy, dx);

			tf2::Quaternion quat_tf;
			quat_tf.setRPY(0, 0, theta); // orientationOffset + PI);

			moveToMapPoint(wallPatternLocationX, wallPatternLocationY-5, quat_tf.z(), quat_tf.w(), 1);
			moveToMapPoint(wallPatternLocationX, wallPatternLocationY, quat_tf.z(), quat_tf.w(), 1);
			state = FINAL;
		}
	}
        else if(state == STACKAPPROACH)
        {
            state = FINAL;
        }
	ROS_INFO("explore AS looped");
        usleep(100000);
    }
	if (state == FINAL) state = SUCCESS; else state = FAIL;
	if (state == SUCCESS) 	server->setSucceeded(result);
	if (state == FAIL) 	server->setAborted(result);
	if (state == PREEMPTED) server->setPreempted(result);
	//state = STOPPING;
    state = IDLE;	
}

void loadPodvod()
{
	ROS_INFO("Loading podvod file");

	std::ifstream loadFile("/home/husky/mbzirc_ws/src/mbzirc_husky/sensor_control/maps/arena/wps.txt");

	int varIdx = 0;
	float x, y;
	while(loadFile >> x >> y)
	{
		ROS_INFO("loaded pos %f %f", x, y);

		if(varIdx == 0)
		{	
			brickStackRedX = x;
			brickStackRedY = y;
		}
		else if(varIdx == 1)
		{
			brickStackOrangeX = x;
			brickStackOrangeY = y;
		}
		else if(varIdx == 2)
		{
			wallPatternLocationKnown = true;
			wallPatternLocationX = x;
			wallPatternLocationY = y;
		}
		else if(varIdx == 3)
		{
			wallPatternOrX = x;
			wallPatternOrY = y;
		}
		varIdx++;
	}
	loadFile.close();
	ROS_INFO("Finished loading podvod file");
	ROS_INFO("%f %f %f %f", brickStackRedX, brickStackRedY, brickStackOrangeX, brickStackOrangeY);
	ROS_INFO("%f %f %f %f", wallPatternLocationX, wallPatternLocationY, wallPatternOrX, wallPatternOrY);
	podvod = true;
}

int main(int argc, char** argv)
{
	loadPodvod();
	ros::init(argc, argv, "brickExplore");
	ros::NodeHandle n;
   	pn = &n;
    movebaseAC = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
    debugVisualiser = n.advertise<visualization_msgs::Marker>("/brickExplore/debug", 1);
    bricksVisualiser = n.advertise<visualization_msgs::Marker>("/brickExplore/bricks", 10);
	// Dynamic reconfiguration server
	dynamic_reconfigure::Server<mbzirc_husky::brick_pileConfig> dynServer;
  	dynamic_reconfigure::Server<mbzirc_husky::brick_pileConfig>::CallbackType f = boost::bind(&dynamicReconfigureCallback, _1, _2);
  	dynServer.setCallback(f);
	prepareClient = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/prepare_gripping");
	symbolicClient = n.serviceClient<mbzirc_husky_msgs::getPoi>("/get_map_poi");
	wprosbagClient = n.serviceClient<std_srvs::Trigger>("/shootVelodyne");
	wallPatternInvestigatorClient = n.serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/raise_camera");
	brickPileDetectorClient = n.serviceClient<mbzirc_husky_msgs::brick_pile_trigger>("/start_brick_pile_detector");
	wallSearchClient = n.serviceClient<mbzirc_husky_msgs::wallPatternDetect>("/searchForWallpattern");
    scan_sub = n.subscribe("/scanlocal",10, scanCallback);	
    //podvod_sub = n.subscribe("/podvod",10, podvodCallback);	
	ransac_pub = n.advertise<std_msgs::String>("ransac/clusterer_reset",1);
	point_pub = n.advertise<sensor_msgs::PointCloud2>("ransac/correct_one_line_f",10);
	point_two_pub = n.advertise<sensor_msgs::PointCloud2>("ransac/correct_two_lines_f",10);
	preciseLocation = n.subscribe("/ransac/clusterer_str_f", 1, preciseLocationCallback);
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
