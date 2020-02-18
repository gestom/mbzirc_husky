#include <signal.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "CTimer.h"
#include "CSegmentation.h"
#include "CTransformation.h"
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
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
#include <ros/console.h>

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
#include <random>
#include <wallpattern_detection/wall_pattern_close.h>
#include <mbzirc_husky_msgs/Float64.h>

// #define PATTERN_DEBUG

using namespace std;
using namespace cv;

VideoCapture *capture;
int key = 0;

int numDetections = 0;
int numDetectionAttempts = 0;
string uav_name;
image_transport::Subscriber imageSub;
ros::Subscriber subHeight, subInfo;
ros::Publisher command_pub;
ros::Publisher posePub;
image_transport::Publisher imagePub;
mbzirc_husky_msgs::wallPatternPosition patternPose;
image_transport::ImageTransport *it;
ros::NodeHandle *n;
int groundPlaneDistance = 0;
random_device rd;

int  defaultImageWidth= 640;
int  defaultImageHeight = 480;
int left_edge = defaultImageWidth/4.0;
int right_edge = defaultImageWidth*(3.0/4.0);
Point2d camera_shift(320, 240);
float cX = defaultImageWidth/2.0;
float cY = defaultImageHeight/2.0;
float fPix = 1.0;
bool got_img;
bool got_height;
bool got_params;

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
tf::TransformListener *listener;
tf::StampedTransform lastTransform;
bool gui;
bool debug = true;
bool oldbags = false;
bool stallImage = false;
int beginX = 0;
int beginY = 0;
ros::Publisher poseArrayPub;
ros::Publisher objectWithTypeArrayPub;
geometry_msgs::PoseArray poseArray;

image_transport::Publisher imdebug;
image_transport::Publisher objectImages;
image_transport::Publisher imhist;
image_transport::Publisher frame_pub;
ros::Publisher pose_pub;
ros::Publisher errorPub;
ros::Publisher objectPublisher;
ros::Subscriber localOdomSub;
ros::Publisher line_pub;

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

static array<int, 2> twoLargest(const int *values, int len, int mid_idx){
    int largestA = -1, largestB = -1;
    int idx1 = -1;
    int idx2 = -1;

    for (int idx = 0; idx < len; idx++) {
        if (idx != mid_idx){
            if(values[idx] >= largestA) {
                largestB = largestA;
                idx2 = idx1;
                largestA = values[idx];
                idx1 = idx;
            } else if (values[idx] >= largestB) {
                largestB = values[idx];
                idx2 = idx;
            }
        }
    }

    if (largestA == 0) idx1 = 0;
    if (largestB == 0) idx2 = 0;
    return { idx1, idx2 };
}

static Point2d transform_using_h(Point2d pt, double fpix, double cx, double cy, double h){
    return Point2d((pt.x -h*cx)/fpix, (pt.y - h*cy)/fpix);
}

array<vector<Point2d>, 3> runRansac3(vector<SSegment> inSegments, int iterations, int hist_size, int bins_num){

    int RANSAC_ITERATIONS = iterations;
    double HIST_SIZE = hist_size;
    int BINS_NUM = bins_num;          /// this has to be odd
    int top_inliers = 0;
    array<Point2d, 2> top_result;
    double area_diff = 500;
    int top_bin_idx = 0;
    int center_bin = int(BINS_NUM/2);
    double max_dist = HIST_SIZE*center_bin - HIST_SIZE/2;           /// beware segfault
    int arr_len = inSegments.size();

    // prepare points and random number generator
    mt19937 rng(rd());
    uniform_int_distribution<int> uni(0, arr_len);
    Point2d all_points[arr_len];
    for (int el = 0; el < arr_len; el++){
        all_points[el] = Point2d(inSegments[el].x, inSegments[el].y);
        /*
        cout << "roundess: " << inSegments[el].roundness << endl;
        cout << "circularity: " << inSegments[el].circularity << endl;
        cout << "size: " << inSegments[el].size << endl;
        */
    }

    for (int it_num = 0; it_num < RANSAC_ITERATIONS; it_num++){
        // init round
        int id1 = uni(rng);
        int id2 = uni(rng);
        if (abs(inSegments[id1].size - inSegments[id2].size) < area_diff){
            int curr_hist[BINS_NUM];
            for (int i = 0; i < BINS_NUM; i++){
                curr_hist[i] = 0;
            }
            Point2d pt1 = all_points[uni(rng)];
            Point2d pt2 = all_points[uni(rng)];
            Point2d vec = pt1 - pt2;
            Point2d norm_vec(-vec.y, vec.x);
            double c = -(norm_vec.x * pt1.x + norm_vec.y * pt1.y);
            Point3d line_eq(norm_vec.x, norm_vec.y, c);
            double line_size = norm(norm_vec);

            // compute all distances a fill the bins
            for (int el = 0; el < arr_len; el++){
                double curr_size = inSegments[el].size;
                Point2d curr_pt = all_points[el];
                double dist = (line_eq.x*curr_pt.x + line_eq.y*curr_pt.y + line_eq.z)/line_size;
                if (dist < max_dist and abs(inSegments[id1].size - inSegments[el].size) < area_diff){
                    int curr_bin = int(round(dist/HIST_SIZE)) + center_bin;
                    curr_hist[curr_bin]++;
                }
            }
            /*
            cout << it_num << ": ";
            for (int i = 0; i < BINS_NUM; i++){
                cout << curr_hist[i] << " ";
            }
            cout << endl;
            */
            // compute inliers
            array<int, 2> two_top_idxs = twoLargest(curr_hist, BINS_NUM, center_bin);
            int idx1 = two_top_idxs[0] - center_bin;
            int idx2 = two_top_idxs[1] - center_bin;
            int inlier_num = curr_hist[center_bin];
            if (idx1 == -idx2 and idx1 != 0){
                /// this is reason why we use histogram - detect 3 colinear lines
                inlier_num += curr_hist[two_top_idxs[0]] + curr_hist[two_top_idxs[1]];
            }

            // set top fits
            if (inlier_num > top_inliers){
                top_inliers = inlier_num;
                top_bin_idx = abs(idx1);
                top_result = {pt1, pt2};
            }
        }
    }

    /// get all inliers

    array<vector<Point2d>, 3> ret;
    // compute distances
    Point2d pt1 = top_result[0];
    Point2d pt2 = top_result[1];
    Point2d vec = pt1 - pt2;
    Point2d norm_vec(-vec.y, vec.x);
    double c = -(norm_vec.x * pt1.x + norm_vec.y * pt1.y);
    Point3d line_eq(norm_vec.x, norm_vec.y, c);
    double line_size = norm(norm_vec);
    for (int el = 0; el < arr_len; el++){
        Point2d curr_pt = all_points[el];
        double dist = (line_eq.x*curr_pt.x + line_eq.y*curr_pt.y + line_eq.z)/line_size;
        if (dist < max_dist){
            int curr_bin = int(round(dist/HIST_SIZE));
            if (curr_bin == -top_bin_idx){
                ret[0].push_back(curr_pt);
            } else if (curr_bin == 0){
                ret[1].push_back(curr_pt);
            } else if (curr_bin == top_bin_idx){
                ret[2].push_back(curr_pt);
            }
        }
    }

    return ret;

}


array<vector<Point2d>, 2> runRansac2(vector<SSegment> inSegments, int iterations, double inlier_dist){

    int RANSAC_ITERATIONS = iterations;
    int top_inliers = 0;
    array<Point2d, 3> top_result;
    double area_diff = 500;
    int top_bin_idx = 0;
    int arr_len = inSegments.size();

    // prepare points and random number generator
    mt19937 rng(rd());
    uniform_int_distribution<int> uni(0, arr_len);
    Point2d all_points[arr_len];
    for (int el = 0; el < arr_len; el++){
        all_points[el] = Point2d(inSegments[el].x, inSegments[el].y);
        /*
        cout << "roundess: " << inSegments[el].roundness << endl;
        cout << "circularity: " << inSegments[el].circularity << endl;
        cout << "size: " << inSegments[el].size << endl;
        */
    }

    for (int it_num = 0; it_num < RANSAC_ITERATIONS; it_num++){
        // init round
        int id1 = uni(rng);
        int id2 = uni(rng);
        if (abs(inSegments[id1].size - inSegments[id2].size) < area_diff){
            int curr_inliers = 0;
            Point2d pt1 = all_points[uni(rng)];
            Point2d pt2 = all_points[uni(rng)];
            Point2d pt3 = all_points[uni(rng)];
            Point2d vec = pt1 - pt2;
            Point2d norm_vec(-vec.y, vec.x);
            double c1 = -(norm_vec.x * pt1.x + norm_vec.y * pt1.y);
            double c2 = -(norm_vec.x * pt3.x + norm_vec.y * pt3.y);
            Point3d line_eq1(norm_vec.x, norm_vec.y, c1);
            Point3d line_eq2(norm_vec.x, norm_vec.y, c2);
            double line_size = norm(norm_vec);

            // compute all distances a fill the bins
            for (int el = 0; el < arr_len; el++){
                double curr_size = inSegments[el].size;
                Point2d curr_pt = all_points[el];
                double dist1 = (line_eq1.x*curr_pt.x + line_eq1.y*curr_pt.y + line_eq1.z)/line_size;
                double dist2 = (line_eq2.x*curr_pt.x + line_eq2.y*curr_pt.y + line_eq2.z)/line_size;
                if ((dist1 < inlier_dist or dist2 < inlier_dist) and abs(inSegments[id1].size - inSegments[el].size) < area_diff){
                    curr_inliers++;
                }
            }

            // set top fits
            if (curr_inliers > top_inliers){
                top_inliers = curr_inliers;
                top_result = {pt1, pt2, pt3};
            }
        }
    }

    /// get all inliers

    array<vector<Point2d>, 2> ret;
    // compute distances
    Point2d pt1 = top_result[0];
    Point2d pt2 = top_result[1];
    Point2d pt3 = top_result[2];
    Point2d vec = pt1 - pt2;
    Point2d norm_vec(-vec.y, vec.x);
    double c1 = -(norm_vec.x * pt1.x + norm_vec.y * pt1.y);
    double c2 = -(norm_vec.x * pt3.x + norm_vec.y * pt3.y);
    Point3d line_eq1(norm_vec.x, norm_vec.y, c1);
    Point3d line_eq2(norm_vec.x, norm_vec.y, c2);
    double line_size = norm(norm_vec);
    for (int el = 0; el < arr_len; el++){
        Point2d curr_pt = all_points[el];
        double dist1 = (line_eq1.x*curr_pt.x + line_eq1.y*curr_pt.y + line_eq1.z)/line_size;
        double dist2 = (line_eq2.x*curr_pt.x + line_eq2.y*curr_pt.y + line_eq2.z)/line_size;
        if (dist1 < inlier_dist){
            ret[0].push_back(curr_pt);
        } else if (dist2 < inlier_dist){
            ret[1].push_back(curr_pt);
        }
    }

    return ret;

}

void magnetHeightCallback(const std_msgs::Float64ConstPtr& msg)
{
    groundPlaneDistance = msg->data*1000+20;
    // printf("Ground plane: %i\n",groundPlaneDistance);
    got_height = true;
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
    cX = msg->K[2];
    cY = msg->K[5];
    fPix = msg->K[0];
    got_params = true;
}

void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
{
    // close detection from above
    if (stallImage == false) inFrame = cv_bridge::toCvShare(msg, "bgr8")->image;
    inFrame.copyTo(frame);

    segmentation.findSeparatedSegment(&frame,&imageCoords,segments,minSegmentSize,maxSegmentSize);
    vector<SSegment> segs_to_ransac;
    int new_size;
    for (int i = 0; i < segmentation.numSegments; i++){
        if (segmentation.segmentArray[i].warning < 1){
            segs_to_ransac.push_back(segmentation.segmentArray[i]);
        }
    }

    if (not segs_to_ransac.empty()){

        array<vector<Point2d>, 3> ret_ransac3 = runRansac3(segs_to_ransac, 500, 10, 35);
        array<vector<Point2d>, 2> ret_ransac2 = runRansac2(segs_to_ransac, 500, 5);

        int r3_sum = ret_ransac3[0].size() + ret_ransac3[1].size() + ret_ransac3[2].size();
        int r2_sum = ret_ransac2[0].size() + ret_ransac2[1].size();

        #ifdef PATTERN_DEBUG
        for (int i = 0; i < segmentation.numSegments; i++){
        drawMarker(frame, Point2d(segmentation.segmentArray[i].x, segmentation.segmentArray[i].y), Scalar(255, 0, 0), MARKER_SQUARE);
        }
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < ret_ransac3[i].size(); j++){
                drawMarker(frame, ret_ransac3[i][j], Scalar(0, 0, 255), MARKER_CROSS);
            }
        }
        drawMarker(frame, Point(segmentation.biggest_segment.x, segmentation.biggest_segment.y), Scalar(0, 255, 0), MARKER_STAR);

        imshow("frame", frame);
        key = waitKey(1)%256;
        #endif

        ROS_INFO_STREAM("Found " << segmentation.numSegments << " segments");
        int lines_num = 0;
        if (ret_ransac3[0].size() > 1 and ret_ransac3[2].size() > 1 and ret_ransac3[1].size() > 3){
            lines_num = 3;
        } else if (ret_ransac2[0].size() > 2 and ret_ransac2[1].size() > 2 and r2_sum > 6) {
            lines_num = 2;
        } else if (ret_ransac3[1].size() > 3){
            lines_num = 1;
        }

        if (got_height and got_img and got_params and lines_num > 0){
            float h = ((groundPlaneDistance - 20) / 1000) + 0.05;
            geometry_msgs::Point pt;
            if (lines_num == 3){
                Point2d pt1 = transform_using_h(ret_ransac3[1][0] - camera_shift, fPix, cX, cY, h);
                Point2d pt2 = transform_using_h(ret_ransac3[1][ret_ransac3[1].size() - 1] - camera_shift, fPix, cX, cY, h);
                Point2d vec = pt2 - pt1;
                Point2d norm_vec = Point2d(-vec.y, vec.x);
                vec = (vec/norm(vec)) * r3_sum;
                double dist = (pt1.x*norm_vec.x + pt1.y*norm_vec.y)/norm(norm_vec);
                pt.x = vec.x;
                pt.y = vec.y;
                pt.z = dist;
            } else if (lines_num == 2){
                Point2d pt1 = transform_using_h(ret_ransac2[0][0] - camera_shift, fPix, cX, cY, h);
                Point2d pt2 = transform_using_h(ret_ransac2[0][ret_ransac2[0].size() - 1] - camera_shift, fPix, cX, cY, h);
                Point2d vec = pt2 - pt1;
                vec = (vec/norm(vec)) * r2_sum;
                pt.x = vec.x;
                pt.y = vec.y;
                pt.z = -1000;
            } else if (lines_num == 1){
                Point2d pt1 = transform_using_h(ret_ransac3[1][0] - camera_shift, fPix, cX, cY, h);
                Point2d pt2 = transform_using_h(ret_ransac3[1][ret_ransac3[1].size() - 1] - camera_shift, fPix, cX, cY, h);
                Point2d vec = pt2 - pt1;
                vec = (vec/norm(vec)) * double(ret_ransac3[1].size());
                pt.x = vec.x;
                pt.y = vec.y;
                pt.z = -1000;
            }
            // cout << "camera params: " << fPix << " " << cX << " " << cY << " " << groundPlaneDistance << endl;

            if (pt.x < 0){
                pt.x = -pt.x;
                pt.y = -pt.y;
                pt.z = -pt.z;
            }
            line_pub.publish(pt);

        }

        if (lines_num == 0){
            Point2d seg_c(segmentation.biggest_segment.x, segmentation.biggest_segment.y);
            if (seg_c.x < left_edge or seg_c.x > right_edge){
                geometry_msgs::Point pt;
                pt.x = 1000;
                pt.y = 1000;
                pt.z = 1000;
                line_pub.publish(pt);
                ROS_INFO_STREAM("End of pattern found!");
            }
        }

    #ifdef PATTERN_DEBUG
        if (gui){
            imshow("frame",frame);
            key = waitKey(1)%256;
	    }
    #endif

    }

    got_img = true;
}

bool getPatternAbove(wallpattern_detection::wall_pattern_close::Request  &req,
                     wallpattern_detection::wall_pattern_close::Response &res){

    if (req.activate){
        #ifndef PATTERN_DEBUG
        ros::ServiceClient raise_arm = n->serviceClient<mbzirc_husky_msgs::Float64>("/kinova/arm_manager/prepare_gripping");
        mbzirc_husky_msgs::Float64 msg;
        msg.request.data = 0.11;
        if (raise_arm.call(msg)){
        #endif
            minSegmentSize = 50;
            /// here comes the code of subscriber
            imageSub = it->subscribe("/camera/color/image_raw", 1, imageCallback2);
            #ifndef PATTERN_DEBUG
            subHeight = n->subscribe("/kinova/arm_manager/camera_to_ground", 1, magnetHeightCallback);
            subInfo = n->subscribe("/camera/color/camera_info", 1, cameraInfoCallback);
            #endif
            line_pub = n->advertise<geometry_msgs::Point>("/wall_pattern_line", 1);

            got_img = false;
            got_height = false;
            got_params = false;

            #ifdef PATTERN_DEBUG
            got_height = true;
            groundPlaneDistance = 0.52;
            got_params = true;
            cX = 327;
            cY = 237;
            fPix = 617;
            #endif

        int attempts = 0;
            while (not(got_img and got_height and got_params) && attempts < 15){
                ros::spinOnce();
                usleep(300000);
                attempts++;
            }
            if (got_img and got_height and got_params){
                ROS_INFO("Service succesfully started subscribers");
                res.success = true;
                return true;
            } else {
                ostringstream s;
                ROS_INFO_STREAM("ERROR - unable to start subscribing" << endl << "subs workin: " << " image " << got_img << " height " << got_height << " cam params " << got_params);
                res.success = false;
                imageSub.shutdown();
                subHeight.shutdown();
                subInfo.shutdown();
                line_pub.shutdown();
                return false;
            }
        #ifndef PATTERN_DEBUG
        } else {
            ROS_INFO("ERROR - unable to raise arm");
            return false;
        }
        #endif
    } else {
        got_img = false;
        got_height = false;
        got_params = false;
        imageSub.shutdown();
        subHeight.shutdown();
        subInfo.shutdown();
        line_pub.shutdown();
        #ifndef PATTERN_DEBUG
        /// TODO !!!!!!!!!!!!!!!!!!
        // ros::ServiceClient fold_arm = n->serviceClient<mbzirc_husky_msgs::Trigger>(); ?????
        #endif
        res.success = true;
        return true;
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if (stallImage == false) inFrame = cv_bridge::toCvShare(msg, "bgr8")->image;
	inFrame.copyTo(frame);
	timer.reset();
	timer.start();

	numDetectionAttempts++;
	SSegment segment = segmentation.findSegment(&frame,&imageCoords,segments,minSegmentSize,maxSegmentSize);
	/*
	   STrackedObject object = altTransform->transform2D(segment);
	   printf("Object: %.2f %.2f %i\n",object.x,object.y,1);
	   */

	if (segment.valid == 1){
		/*pZ = segment.z/1000;
		  pX = (segment.x-cX)/fPix*pZ+cameraXOffset+cameraXAngleOffset*pZ;
		  pY = (segment.y-cY)/fPix*pZ+cameraYOffset+cameraXAngleOffset*pZ;

		  patternPose.pose.pose.position.x = pX;
		  patternPose.pose.pose.position.y = pY;
		  patternPose.pose.pose.position.z = pZ;
		  patternPose.type = segment.type;
		  tf2::Quaternion quat_tf;
		  quat_tf.setRPY(0,0,segment.angle);
		  patternPose.pose.pose.orientation = tf2::toMsg(quat_tf);
		  patternPose.detected = true;
		  patternPose.completelyVisible = (segment.warning == false);*/
		numDetections++;
	}
	// posePub.publish(patternPose);
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

bool detect(mbzirc_husky_msgs::wallPatternDetect::Request  &req, mbzirc_husky_msgs::wallPatternDetect::Response &res)
{

	if (req.activate)
	{
		imageSub = it->subscribe("/camera/color/image_raw", 1, &imageCallback);
		subHeight = n->subscribe("/kinova/arm_manager/camera_to_ground", 1, magnetHeightCallback);
		subInfo = n->subscribe("/camera/color/camera_info", 1, cameraInfoCallback);

		groundPlaneDistance = req.groundPlaneDistance;
		numDetections = 0;
		numDetectionAttempts = 0;
		int attempts = 0;
		while (numDetectionAttempts == 0 && attempts < 20){
			ros::spinOnce();
			usleep(100000);
			attempts++;
		}
		if (numDetections > 0){
			ROS_INFO("Brick detected.");
			res.patternPose = patternPose.pose;
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
	} else
	{
		res.detected = false;
		res.activated = false;
		subHeight.shutdown();
		imageSub.shutdown();
		subInfo.shutdown();
	}
	return true;
}


int main(int argc, char** argv) 
{
	offset.x = offset.y = diffPose.x = diffPose.y = 0;

	ros::init(argc, argv, "wallpattern_detector");
	n = new ros::NodeHandle();
	it = new image_transport::ImageTransport(*n);
	string clr_filepath = ros::package::getPath("wallpattern_detection") + "/etc/rosbag.bin";

	n->param("camera_yaw_offset", camera_yaw_offset, 0.0);
	n->param("camera_phi_offset", camera_phi_offset, 0.0);
	n->param("camera_psi_offset", camera_psi_offset, 0.0);
	n->param("max_segment_size", maxSegmentSize, 1000);
	n->param("camera_delay", camera_delay, 0.0);
	n->param("camera_offset", camera_offset, 0.17);
	n->param("wallpattern_height", wallpattern_height, 0.20);
	// n->param("colormap_filename", colormap_filename, std::string("rosbag.bin"));
    n->param("colormap_filename", colormap_filename, clr_filepath);
    n->param("uav_name", uav_name, string());
    n->param("gui", gui, false);
    n->param("debug", debug, false);

    // gui = true;

    if (gui) {
        debug = true;
        signal (SIGINT,termHandler);
    }

	if (gui) namedWindow("frame", CV_WINDOW_AUTOSIZE);
	if (gui) namedWindow("histogram", CV_WINDOW_AUTOSIZE);
	if (gui) namedWindow("roi", CV_WINDOW_AUTOSIZE);

	colorMap = colormap_filename;
	segmentation.loadColorMap(colorMap.c_str());
	segmentation.loadColors((colorMap+".col").c_str());

	/*  TODO: get calibration file
	altTransform = new CTransformation();
	string calibrationFile = ros::package::getPath("wallpattern_detection")+"/etc/correspondences.col";
	altTransform->calibrate2D(calibrationFile.c_str());
	*/

    ros::ServiceServer service2 = n->advertiseService("start_top_wall_detector", getPatternAbove);

	// initialize dynamic reconfiguration feedback

	dynamic_reconfigure::Server<wallpattern_detection::wallpattern_detectionConfig> server;
	dynamic_reconfigure::Server<wallpattern_detection::wallpattern_detectionConfig>::CallbackType dynSer;
	dynSer = boost::bind(&reconfigureCallback, _1, _2);
	server.setCallback(dynSer);

		// SUBSCRIBERS
    ros::ServiceServer service;
    if (gui){
        service = n->advertiseService("detectWallpattern", detect);
        imagePub = it->advertise("/wallDetectResult", 1);
        ros::Subscriber subGrasp = n->subscribe("grasping_result", 1, &graspCallback, ros::TransportHints().tcpNoDelay());
    }

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
