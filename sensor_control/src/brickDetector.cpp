#include <ros/ros.h>
#include "CTimer.h"
#include "CRawImage.h"
#include "CRawDepthImage.h"
#include "CSegmentation.h"
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mbzirc_husky_msgs/brickDetect.h>
#include <mbzirc_husky/detectBrickConfig.h>
#include <mbzirc_husky_msgs/brickPosition.h>

int numDetections = 0;
int numDetectionAttempts = 0;
SSegment segment;
mbzirc_husky_msgs::brickPosition brickPose;
image_transport::Publisher imdebug;
ros::Publisher command_pub;
ros::Publisher posePub;
image_transport::Publisher imagePub;
image_transport::Subscriber subimDepth;
image_transport::Subscriber subimColor;
ros::Subscriber subHeight;
image_transport::ImageTransport *it;
ros::NodeHandle *n;

CSegmentation *segmentation;
CRawImage *colorImage;
CRawDepthImage *depthImage;

int  defaultImageWidth= 640;
int  defaultImageHeight = 480;
int groundPlaneDistance = 0;
int wantedType = 0;
float cameraXOffset = -0.02;
float cameraYOffset = -0.02;
float cameraXAngleOffset = 0;
float cameraYAngleOffset = 0;
//parameter reconfiguration
void reconfigureCallback(mbzirc_husky::detectBrickConfig &config, uint32_t level) 
{
	cameraXOffset = config.cameraXOffset;
	cameraYOffset = config.cameraYOffset;
	ROS_INFO("Reconfigure Request: %lf %lf", cameraXOffset,cameraYOffset);
}


void colorImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if (colorImage->bpp != msg->step/msg->width || colorImage->width != msg->width || colorImage->height != msg->height){
		delete colorImage;
		ROS_DEBUG("Readjusting colorImage format from %ix%i %ibpp, to %ix%i %ibpp.",colorImage->width,colorImage->height,colorImage->bpp,msg->width,msg->height,msg->step/msg->width);
		colorImage = new CRawImage(msg->width,msg->height,msg->step/msg->width);
	}
	memcpy(colorImage->data,(void*)&msg->data[0],msg->step*msg->height);
}

void magnetHeightCallback(const std_msgs::Float64ConstPtr& msg)
{
	groundPlaneDistance = msg->data*1000+20;
	printf("Ground plane: %i\n",groundPlaneDistance);
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	segment.valid = 0;
	numDetectionAttempts++;
	if (depthImage->bpp != msg->step/msg->width || depthImage->width != msg->width || depthImage->height != msg->height)
	{
		delete depthImage;
		ROS_INFO("Readjusting depthImage format from %ix%i %ibpp, to %ix%i %ibpp.",depthImage->width,depthImage->height,depthImage->bpp,msg->width,msg->height,msg->step/msg->width);
		depthImage = new CRawDepthImage(msg->width,msg->height,msg->step/msg->width);
	}
	memcpy(depthImage->data,(void*)&msg->data[0],msg->step*msg->height);
	depthImage->getClosest(groundPlaneDistance);
	segment = segmentation->findSegment(depthImage,13000,10000000,wantedType,colorImage);
	float pX,pY,pZ;
	brickPose.detected = false;
	brickPose.completelyVisible = false;
	if (segment.valid == 1){
		pZ = segment.z/1000;
		pX = (segment.x-320.81)/388.33*pZ+cameraXOffset+cameraXAngleOffset*pZ;
		pY = (segment.y-243.82)/388.33*pZ+cameraYOffset+cameraXAngleOffset*pZ;
		brickPose.pose.pose.position.x = pX;
		brickPose.pose.pose.position.y = pY;
		brickPose.pose.pose.position.z = pZ;
		brickPose.type = segment.type;
		tf2::Quaternion quat_tf;
		quat_tf.setRPY(0,0,segment.angle);
		brickPose.pose.pose.orientation = tf2::toMsg(quat_tf);
		brickPose.detected = true;
		brickPose.completelyVisible = (segment.warning == false);
		numDetections++; 
	}
	posePub.publish(brickPose);
	if (imagePub.getNumSubscribers() != 0){
		sensor_msgs::Image outputImage;
		outputImage.header.stamp     = ros::Time::now();
		outputImage.height           = depthImage->height;
		outputImage.width            = depthImage->width;
		outputImage.encoding         = "rgb8";
		outputImage.is_bigendian     = false;
		outputImage.step             = depthImage->width*3;
		unsigned char *buffer = (unsigned char*)calloc(depthImage->size*3,sizeof(unsigned char));
		depthImage->generateRGB(buffer);
		for(int i=0; i<depthImage->size;i++)
		{
			outputImage.data.push_back(buffer[3*i+0]);
			outputImage.data.push_back(buffer[3*i+1]);
			outputImage.data.push_back(buffer[3*i+2]);
		}
		imagePub.publish(outputImage);
		free(buffer);
	}
}

bool detect(mbzirc_husky_msgs::brickDetect::Request  &req, mbzirc_husky_msgs::brickDetect::Response &res)
{

	if (req.activate){
		segmentation->resetTracking(depthImage,req.x,req.y);
	       	subimDepth = it->subscribe("/camera/depth/image_rect_raw", 1, depthImageCallback);
	       	subimColor = it->subscribe("/camera/color/image_raw", 1, colorImageCallback);
		subHeight = n->subscribe("/kinova/arm_manager/camera_to_ground", 1, magnetHeightCallback);
		groundPlaneDistance = req.groundPlaneDistance;
		numDetections = 0;
		numDetectionAttempts = 0;
		wantedType = req.wantedType;
		int attempts = 0;
		while (numDetectionAttempts == 0 && attempts < 20){
			ros::spinOnce();
			usleep(100000);
			attempts++;
		}
		if (numDetections > 0){
			ROS_INFO("Brick detected.");
			res.brickPose = brickPose.pose;
			res.detected = true;
			res.activated = true;
		}else if (numDetectionAttempts > 0){
			ROS_INFO("Brick not detected.");
			res.detected = false;
			res.activated = true;
		}else{
			ROS_INFO("Depth image not incoming. Is realsense on?");	
			res.detected = false;
			res.activated = false;
		}
	}else
	{
		res.detected = false;
		res.activated = false;
	       	subimDepth.shutdown();
		subHeight.shutdown();
		subimColor.shutdown();
	}
	return true;
}


int main(int argc, char** argv)
{
	segmentation = new CSegmentation();
	ros::init(argc, argv, "brickDetector");
	n = new ros::NodeHandle();
	it = new image_transport::ImageTransport(*n);
	colorImage = new CRawImage(defaultImageWidth,defaultImageHeight,4);
	depthImage = new CRawDepthImage(defaultImageWidth,defaultImageHeight,4);
	imagePub = it->advertise("/image_with_features", 1);
	ros::ServiceServer service = n->advertiseService("detectBricks", detect);

	//initialize dynamic reconfiguration feedback
	dynamic_reconfigure::Server<mbzirc_husky::detectBrickConfig> server;
	dynamic_reconfigure::Server<mbzirc_husky::detectBrickConfig>::CallbackType dynSer = boost::bind(&reconfigureCallback, _1, _2);
	server.setCallback(dynSer);

	posePub = n->advertise<mbzirc_husky_msgs::brickPosition>("/brickPosition", 1);

	while (ros::ok()){
		ros::spinOnce();
		usleep(30000);
	}
	ros::shutdown();
}

