#include <ros/ros.h>
#include "CTimer.h"
#include "CRawImage.h"
#include "CRawDepthImage.h"
#include "CSegmentation.h"
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

image_transport::Publisher imdebug;
ros::Publisher command_pub;
ros::Publisher posePub;
image_transport::Publisher imagePub;

CSegmentation *segmentation;
CRawImage *grayImage;
CRawDepthImage *depthImage;

int  defaultImageWidth= 640;
int  defaultImageHeight = 480;

//parameter reconfiguration
/*void reconfigureCallback(social_card_reader::social_cardConfig &config, uint32_t level) 
{
	ROS_INFO("Reconfigure Request: %lf %lf %lf %lf %lf %lf %lf", config.userDiameter, config.masterDiameter, config.initialCircularityTolerance, config.finalCircularityTolerance, config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs);
	outerDimUser = config.userDiameter/100.0;
	outerDimMaster = config.masterDiameter/100.0;
	distanceTolerance = config.distanceTolerance/100.0;
	detector->reconfigure(config.initialCircularityTolerance, config.finalCircularityTolerance, config.areaRatioTolerance,config.centerDistanceToleranceRatio,config.centerDistanceToleranceAbs);
}*/


void grayImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if (grayImage->bpp != msg->step/msg->width || grayImage->width != msg->width || grayImage->height != msg->height){
		delete grayImage;
		ROS_DEBUG("Readjusting grayImage format from %ix%i %ibpp, to %ix%i %ibpp.",grayImage->width,grayImage->height,grayImage->bpp,msg->width,msg->height,msg->step/msg->width);
		grayImage = new CRawImage(msg->width,msg->height,msg->step/msg->width);
	}
	memcpy(grayImage->data,(void*)&msg->data[0],msg->step*msg->height);
	}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if (depthImage->bpp != msg->step/msg->width || depthImage->width != msg->width || depthImage->height != msg->height)
	{
		delete depthImage;
		ROS_INFO("Readjusting depthImage format from %ix%i %ibpp, to %ix%i %ibpp.",depthImage->width,depthImage->height,depthImage->bpp,msg->width,msg->height,msg->step/msg->width);
		depthImage = new CRawDepthImage(msg->width,msg->height,msg->step/msg->width);
	}
	memcpy(depthImage->data,(void*)&msg->data[0],msg->step*msg->height);
	depthImage->getClosest(100);
	SSegment segment = segmentation->findSegment(depthImage,5000,10000000);
	float pX,pY,pZ;
	if (segment.valid == 1){
		pZ = segment.z/1000;	
		pX = (segment.x-307)/640.95*pZ;
		pY = (segment.y-243.12)/640.95*pZ;
		geometry_msgs::PoseStamped pose;
		pose.pose.position.x = pX;
		pose.pose.position.y = pY;
		pose.pose.position.z = pZ;
		tf2::Quaternion quat_tf;
		quat_tf.setRPY(0,0,M_PI/2);
		pose.pose.orientation = tf2::toMsg(quat_tf);
		posePub.publish(pose);
	}
	printf("Pos: %i %f %f %f\n",segment.valid,pX,pY,pZ);
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

int main(int argc, char** argv)
{
	segmentation = new CSegmentation();
	ros::init(argc, argv, "brickDetector");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	grayImage = new CRawImage(defaultImageWidth,defaultImageHeight,4);
	depthImage = new CRawDepthImage(defaultImageWidth,defaultImageHeight,4);
	imagePub = it.advertise("/image_with_features", 1);

	//initialize dynamic reconfiguration feedback
//	dynamic_reconfigure::Server<social_card_reader::social_cardConfig> server;
//	dynamic_reconfigure::Server<social_card_reader::social_cardConfig>::CallbackType dynSer;
//	dynSer = boost::bind(&reconfigureCallback, _1, _2);
//	server.setCallback(dynSer);

//	photoTf = new CTransformation(outerDimUser);
//	commandTf = new CTransformation(outerDimMaster);
	//image_transport::Subscriber subimGray = it.subscribe("/head_xtion/rgb/image_mono", 1, grayImageCallback);
	image_transport::Subscriber subimDepth = it.subscribe("/camera/depth/image_rect_raw", 1, depthImageCallback);
	posePub = n.advertise<geometry_msgs::PoseStamped>("/brickPosition", 1);
	//command_pub = n.advertise<std_msgs::String>("/socialCardReader/commands", 1);

	while (ros::ok()){
		ros::spinOnce();
		usleep(30000);
	}
}

