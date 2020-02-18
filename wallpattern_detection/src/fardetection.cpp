#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "CTimer.h"
#include "CSegmentation.h"
#include "CTransformation.h"
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <mbzirc_husky_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <dynamic_reconfigure/server.h>
#include <wallpattern_detection/wallpattern_detectionConfig.h>
#include <wallpattern_detection/detectedobject.h>
#include <wallpattern_detection/ObjectWithType.h>
#include <wallpattern_detection/ObjectWithTypeArray.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include "opencv2/ml/ml.hpp"
#include <mbzirc_husky_msgs/wallPatternDetect.h>
#include <wallpattern_detection/wallpattern_detectionConfig.h>
#include <mbzirc_husky_msgs/wallPatternPosition.h>
#include <mbzirc_husky_msgs/Gen3ArmStatus.h>

#include <mbzirc_husky/setPoi.h>
#include <mbzirc_husky/getPoi.h>

#include <sys/time.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace cv;

VideoCapture *capture;
int key = 0;

geometry_msgs::PoseStamped anchorPose;
geometry_msgs::PoseStamped robotPose;
tf::TransformListener *listener;
int numDetections = 0;
int detectionStrength = 0;
int numDetectionAttempts = 0;
string uav_name;
image_transport::Subscriber imageSub;
ros::Subscriber subInfo;
ros::Subscriber armInfo;
ros::Publisher command_pub;
ros::Publisher posePub;
image_transport::Publisher imagePub;
mbzirc_husky_msgs::wallPatternPosition patternPose;
image_transport::ImageTransport *it;
ros::NodeHandle *n;

int  defaultImageWidth= 640;
int  defaultImageHeight = 480;
float cX = defaultImageWidth/2.0;
float cY = defaultImageHeight/2.0;
float fPix = 1.0;
bool gotCameraInfo = false;
float armAngle = 0;

String colorMap;
int segmentType = 1;
int objectImageSequence = 0;
int imageNumber = 0;
float satur = 1.0;
float minRoundness = 0.02;
Mat gaussianSamples;
Mat storedSamples;
ros::Time lastTime;
geometry_msgs::PoseStamped lastPose;
geometry_msgs::Point diffPose;
geometry_msgs::Point offset;

MatND histogram;
int hbins = 180;
int sbins = 256;
tf::StampedTransform lastTransform;
bool gui = true;
bool debug = true;
bool oldbags = false;
bool stallImage = false;
int beginX = 0;
int beginY = 0;
ros::Publisher poseArrayPub;
ros::Publisher objectWithTypeArrayPub;
geometry_msgs::PoseArray poseArray;
ros::ServiceClient armClient;

image_transport::Publisher imdebug;
image_transport::Publisher objectImages;
image_transport::Publisher imhist;
image_transport::Publisher frame_pub;
ros::Publisher pose_pub;
ros::Publisher errorPub;
ros::Publisher objectPublisher;
ros::Subscriber localOdomSub;

SSegment currentSegment;
SSegment lastSegment;

CSegmentation segmentation;
CTransformation *altTransform;
Mat videoFrame,frama,distorted;
Mat imageCoords,metricCoords;
CTimer timer;
int frameExists = 0;
int detectedObjects = 0;
Mat mask,frame,rframe,inFrame;
SSegment segments[MAX_SEGMENTS];
wallpattern_detection::detectedobject objectDescriptionArray[MAX_SEGMENTS];

// LATER FILL FROM CONFIG FILE
Mat distCoeffs = Mat_<double>(1,5);	
Mat intrinsic = Mat_<double>(3,3);	

cv_bridge::CvImage cv_ptr;

int minSegmentSize = 10;
int maxSegmentSize = 1000000;
int circleDetections = 0;
int maxCircleDetections = 10;
int numCommands = 5;
int lastCommand = -1;
int command = -1;
float angle = 0;
float distanceTolerance = 0.2;
float outerDimUser = 0.05;
float outerDimMaster = 0.07;
int outputImageIndex = 0;
int manualThreshold = 50;
double camera_yaw_offset = 0;
double camera_phi_offset = 0;
double camera_psi_offset = 0;
double camera_delay = 0.22;
double wallpattern_height = 0.20;
double camera_offset = 0.17;
double circleDiameter = 0.2;
int histogramScale = 3;
float visualDistanceToleranceRatio = 0.2;
float visualDistanceToleranceAbsolute =  0.2;
bool longObjectDetection = false;

std::string colormap_filename;

//parameter reconfiguration
void reconfigureCallback(wallpattern_detection::wallpattern_detectionConfig &config, uint32_t level) 
{
	// ROS_INFO("Reconfigure Request: %lf %lf %lf %lf %lf %lf %lf", config.outputImageIndex);
	outputImageIndex = config.outputImageIndex;
	manualThreshold = config.manualThreshold;
	maxSegmentSize = config.maxBlobSize;
	minSegmentSize = config.minBlobSize;
	segmentation.minCircularity = config.minCircularity;
	minRoundness = config.minRoundness;
	circleDiameter = config.objectDiameter;
	histogramScale = config.histogramScale;
	visualDistanceToleranceRatio = config.visualDistanceToleranceRatio;
	visualDistanceToleranceAbsolute = config.visualDistanceToleranceAbsolute;
	longObjectDetection = config.longObject;
	//  ROS_INFO("Reconfigure Request min circularity, min convexity: %lf %lf %lf", config.minCircularity, config.manualThreshold,circleDiameter);
	//outerDimMaster = config.masterDiameter/100.0;
	//distanceTolerance = config.distanceTolerance/100.0;
	//detector->reconfigure(config.initialCircularityTolerance, config.finalCircularityTolerance, config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs);
}

void learnSegments(int number);

void graspCallback(const std_msgs::Int32 &msg)
{
  if (msg.data == -1 || msg.data == -2){
    offset.x = diffPose.x*0.5;
    offset.y = diffPose.y*0.5;
  }else{
    offset.x = 0;
    offset.y = 0;
  }
}

void saveColors()
{
	if (detectedObjects == 4){
		float ix[] = {-1,-0,+1,+0};
		float iy[] = {+0,-1,-0,+1};
		int ii[4];
		string filename = ros::package::getPath("wallpattern_detection")+"/etc/"+colormap_filename+".col";
		FILE *file = fopen(filename.c_str(),"w+");
		for (int i = 0;i<detectedObjects;i++)
		{
			int index = 0;
			float minEval = -100000;
			float eval = -100000;
			for (int j = 0;j<4;j++){
				eval = 	objectDescriptionArray[j].x*ix[i] + objectDescriptionArray[j].y*iy[i];
				if (eval > minEval){
					minEval = eval;
					index = j;
				}
			}
			int ii = i;
			if (ii == 0) ii = 4;
			fprintf(file,"%i %.0f %.0f %.0f\n",ii,objectDescriptionArray[index].h,objectDescriptionArray[index].s,objectDescriptionArray[index].v);
			segmentation.setColor(ii,objectDescriptionArray[index].h,objectDescriptionArray[index].s,objectDescriptionArray[index].v);
		}
		fclose(file);
	}
}

STrackedObject transformPatternPose(STrackedObject object)
{
	int inc = 0;
	geometry_msgs::PoseStamped resultPose,pose;
	STrackedObject result = object;
	float az = 0;
	try {
		float x,y;
		float armOffsetX = 0;
		float armOffsetY = 0;
		x = + cos(armAngle)*object.x + sin(armAngle)*object.y+armOffsetX;
		y = - sin(armAngle)*object.x + cos(armAngle)*object.y+armOffsetY;
		pose.header.frame_id = "base_link";
		pose.header.stamp = ros::Time::now();
		pose.pose.position.x = x;
		pose.pose.position.y = y;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		listener->waitForTransform("/base_link","map",pose.header.stamp,ros::Duration(0.2));
		listener->transformPose("/map",pose,resultPose);
		result.x = resultPose.pose.position.x; 
		result.y = resultPose.pose.position.y;
		printf("Main object: %.2f %.2f %.2f %.2f %.2f %.2f %i %i %.2f\n",result.x,result.y,x,y,object.x,object.y,object.numContours,numDetectionAttempts,armAngle);
		if (result.numContours > 1) reportPosition(result);
		return result;
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		result.validity = 0;
		return result;
	}
}

int reportPosition(STrackedObject object)
{
	    mbzirc_husky::setPoi poi;
            poi.request.type = 3;
            poi.request.x = object.x;
            poi.request.y = object.y;
            poi.request.covariance = object.numContours;
            if (ros::service::call("set_map_poi", poi)){
                cout << "Wall pattern sent to symbolic map" << endl;
            } else {
                cout << "Error calling service" << endl;
            }
}

int updateRobotPosition()
{
	int inc = 0;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped tf_pose;
	float az = 0;
	try {
		pose.header.frame_id = "base_link";
		pose.header.stamp = ros::Time::now();
		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 0;
		pose.pose.orientation.x = 0;
		pose.pose.orientation.y = 0;
		pose.pose.orientation.z = 0;
		pose.pose.orientation.w = 1;
		listener->waitForTransform("/base_link","map",pose.header.stamp,ros::Duration(0.2));
		listener->transformPose("/map",pose,robotPose);
		return 0;
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		return -1;
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if (stallImage == false) inFrame = cv_bridge::toCvShare(msg, "bgr8")->image;
	inFrame.copyTo(frame);
	timer.reset();
	timer.start();
	updateRobotPosition();
	numDetectionAttempts++;
	SSegment segment = segmentation.findSegment(&frame,&imageCoords,segments,minSegmentSize,maxSegmentSize);
	if (segment.valid == 1){
		STrackedObject object = altTransform->transform2D(segment);
		object = transformPatternPose(object);
		detectionStrength += object.numContours;  
		numDetections++;
	}
	//posePub.publish(patternPose);

	if (imagePub.getNumSubscribers() != 0){
		frame.copyTo(videoFrame);
		cv_ptr.encoding = "bgr8";
		cv_ptr.image = videoFrame;
		sensor_msgs::ImagePtr imagePtr2 = cv_ptr.toImageMsg();
		imagePtr2->header.seq = cv_bridge::toCvShare(msg, "bgr8")->header.seq;

		try {
			frame_pub.publish(imagePtr2);
		} catch (...) {
			ROS_ERROR("Exception caught during publishing topic %s.", frame_pub.getTopic().c_str());
		}
	}
	//END of segmentation - START calculating global frame coords
	if (gui){
		imshow("frame",frame);

		/*processing user input*/
		key = waitKey(1)%256;
		if (key == 32) stallImage = !stallImage;
		printf("STALL %i\n",stallImage);
		if (key == 'r'){
			segmentation.resetColorMap();
			histogram = Mat::zeros(hbins,sbins,CV_32FC1);
			storedSamples = Mat::zeros(0,3,CV_32FC1);
		}
		if (key == 's') segmentation.saveColorMap(colorMap.c_str());
		if (key == 'c') saveColors();
		if (key >= '1' && key < '9') segmentType = (key-'0');
	}
	imageNumber++;
}

void mainMouseCallback(int event, int x, int y, int flags, void* userdata)
{
	//start dragging - starting to define region of interrest
	if  ( event == EVENT_LBUTTONDOWN || event == EVENT_RBUTTONDOWN)
	{
		beginX = x;
		beginY = y;
	}
	//end dragging -  region of interrest defined
	if  ( (event == EVENT_LBUTTONUP || event == EVENT_LBUTTONUP))
	{
		//reverse positions of x/beginX and y/beginY if mouse drag in opposite direction 
		if (beginX > x){int tmpx = x;x=beginX;beginX = tmpx;}
		if (beginY > y){int tmpy = y;y=beginY;beginY = tmpy;}

		//cut off a region of interrest and convert to HSV color space
		Mat sample = Mat::zeros(1,3,CV_32FC1);
		Mat roi = frame(Rect(beginX,beginY ,x-beginX+1,y-beginY+1));
		Mat hsv(roi.rows,roi.cols,CV_8UC3);
		cvtColor(roi,hsv, COLOR_RGB2HSV,1);

		//create a list of pixels with given colors  
		for( int iy = 0; iy < hsv.rows; iy++ ){
			for( int ix = 0; ix < hsv.cols; ix++ ){
				Vec3b pixel = hsv.at<Vec3b>(iy,ix);
				for( int i = 0; i < 3;i++ ) sample.at<float>(i) = pixel(i);
				storedSamples.push_back(sample);
			}
		}

		//fill the histogram with the color samples   
		Mat histogram = Mat::zeros(180,256, CV_32FC1);
		for( int i = 0; i < storedSamples.rows; i++ ) histogram.at<float>((int)storedSamples.at<float>(i,0),(int)storedSamples.at<float>(i,1))++;

		//fill the histogram with the samples   
		double maxVal=0;
		histogram.at<float>(0,0)=0;
		minMaxLoc(histogram, 0, &maxVal, 0, 0);
		int scale = 1;
		Mat histImg = Mat::zeros(histogramScale*180, histogramScale*256, CV_8UC3);

		for( int h = 0; h < histogramScale*180; h++ ){
			for( int s = 0; s < histogramScale*256; s++ ){
				float binVal = histogram.at<float>(h/histogramScale, s/histogramScale);
				int intensity = cvRound(logf(binVal)*255/logf(maxVal));
				Vec3b pixel = Vec3b(intensity,intensity,intensity);
				histImg.at<Vec3b>(h,s) = pixel;
			}
		}

		if (gui) imshow("roi",roi);
		if (gui) imshow("histogram",histImg);

		cout << frame.at<Vec3b>(y,x) << endl;
	}
}

void histogramMouseCallback(int event, int x, int y, int flags, void* userdata)
{
  if  ( event == EVENT_LBUTTONDOWN )
  {
    beginX = x;
    beginY = y;
  }
  if  ( event == EVENT_LBUTTONUP )
  {
	  beginX = beginX/histogramScale;
	  beginY = beginY/histogramScale;
	  x=x/histogramScale;
	  y=y/histogramScale;
	  segmentation.learnPixel(beginY-2,y+2,beginX-2,x+2,50,255,segmentType);
  }
}

//to speed up termination
void termHandler(int s){
  exit(1); 
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
	gotCameraInfo = true;
	cX = msg->K[2];
	cY = msg->K[5];
	fPix = msg->K[0];
}
 
void armStatusCallback(const mbzirc_husky_msgs::Gen3ArmStatusConstPtr& msg)
{
	armAngle = msg->joint_angles[4];
}


bool detect(mbzirc_husky_msgs::wallPatternDetect::Request  &req, mbzirc_husky_msgs::wallPatternDetect::Response &res)
{
	subInfo = n->subscribe("/camera/color/camera_info", 1, cameraInfoCallback);
	mbzirc_husky_msgs::Float64 srv;
	srv.request.data = -M_PI/2;
	armClient.call(srv);
	for (float i = -1;i<1;i+=1.0/3)
	{
		srv.request.data = i*M_PI;
		armClient.call(srv);
		numDetections = 0;
		numDetectionAttempts = 0;
		detectionStrength = 0;
		imageSub = it->subscribe("/camera/color/image_raw", 1, &imageCallback);
		while (numDetectionAttempts < 60){
			ros::spinOnce();
		}
		imageSub.shutdown();
	}
	srv.request.data = M_PI/2;
	armClient.call(srv);
	srv.request.data = 0;
	armClient.call(srv);
	subInfo.shutdown();
	return true;
}


int main(int argc, char** argv) 
{
	offset.x = offset.y = diffPose.x = diffPose.y = 0;
	ros::init(argc, argv, "wallpattern_detector");
	n = new ros::NodeHandle();
	it = new image_transport::ImageTransport(*n);

	n->param("uav_name", uav_name, string());
	n->param("gui", gui, true);
	n->param("debug", debug, false);
	if (gui) {
		debug = true;
		signal (SIGINT,termHandler);
	}

	n->param("camera_yaw_offset", camera_yaw_offset, 0.0);
	n->param("camera_phi_offset", camera_phi_offset, 0.0);
	n->param("camera_psi_offset", camera_psi_offset, 0.0);
	n->param("max_segment_size", maxSegmentSize, 1000);
	n->param("camera_delay", camera_delay, 0.0);
	n->param("camera_offset", camera_offset, 0.17);
	n->param("wallpattern_height", wallpattern_height, 0.20);
	n->param("colormap_filename", colormap_filename, std::string("rosbag.bin"));
	
	if (gui) namedWindow("frame", CV_WINDOW_AUTOSIZE);
	if (gui) namedWindow("histogram", CV_WINDOW_AUTOSIZE);
	if (gui) namedWindow("roi", CV_WINDOW_AUTOSIZE);

	colorMap = ros::package::getPath("wallpattern_detection")+"/etc/farDetect.colormap";
	segmentation.loadColorMap(colorMap.c_str());
	segmentation.loadColors((colorMap+".col").c_str());

	altTransform = new CTransformation();


	string calibrationFile = ros::package::getPath("wallpattern_detection")+"/etc/correspondences.cal";
	altTransform->calibrate2D(calibrationFile.c_str());
	ros::ServiceServer service = n->advertiseService("searchForWallpattern", detect);
	armClient       = n->serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/raise_camera");
	imagePub = it->advertise("/searchWallResult", 1);

	// initialize dynamic reconfiguration feedback
	dynamic_reconfigure::Server<wallpattern_detection::wallpattern_detectionConfig> server;
	dynamic_reconfigure::Server<wallpattern_detection::wallpattern_detectionConfig>::CallbackType dynSer;
	dynSer = boost::bind(&reconfigureCallback, _1, _2);
	server.setCallback(dynSer);
	armInfo = n->subscribe("/kinova/arm_manager/status", 1, armStatusCallback);
	listener = new tf::TransformListener();

	// Debugging PUBLISHERS
	if (debug) {
		poseArrayPub = n->advertise<geometry_msgs::PoseArray>("objectPositions", 1);
		pose_pub = n->advertise<geometry_msgs::PoseStamped>("objectRelative", 1);
		imdebug = it->advertise("processedimage", 1);
	}

	sensor_msgs::Image msg;
	if (gui) setMouseCallback("frame", mainMouseCallback, NULL);
	if (gui) setMouseCallback("histogram", histogramMouseCallback, NULL);

	timer.reset();
	timer.start();

	ros::spin();
}

