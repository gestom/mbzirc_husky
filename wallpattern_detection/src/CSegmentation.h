/*
 * File name: CSegmentation.h
 * Date:      2010
 * Author:   Tom Krajnik 
 */

#ifndef __CSEGMENATION_H__
#define __CSEGMENATION_H__

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <stdio.h>
#define MAX_SEGMENTS 10000
#define MIN_SEGMENT_SIZE 40 
#define MAX_CONTOUR_POINTS 1000

using namespace std;
using namespace cv;


typedef struct{
	float x,y,h,s,v;
	float minX, minY,maxX,maxY;
	float px,py;
	float v0,v1;
	float m0,m1;
	float ch,cs,cv;
	int cornerX[4];
	int cornerY[4];
	int contourX[MAX_CONTOUR_POINTS];
	int contourY[MAX_CONTOUR_POINTS];
	int contourPoints;
	int contours;
	float crit;
	int id;
	int size;
	int type;
	int combo;
	int warning;
	int valid;
	float roundness;
	float circularity;
}SSegment;

class CSegmentation
{
	public:
		CSegmentation();
		~CSegmentation();
		SSegment findSegment(Mat* image,Mat *coords,SSegment *output,int minSegmentSize,int maxSegmentSize);
		SSegment findSeparatedSegment(Mat* image,Mat *coords,SSegment *output,int minSegmentSize,int maxSegmentSize);

		SSegment separateContours(int *image,Mat *coords,SSegment *output,int minSize,int maxSize);
		void setColor(int i,float h,float s,float v);
		SSegment getSegment(int type,int number);
		void learnPixel(Vec3b a,int type = 1);
		void learnPixel(unsigned char* a,int type = 1);
		void learnPixel(int minHue,int maxHue,int minSat,int maxSat,int minVal,int maxVal,int value = 1);
		int loadColors(const char* name);
		void setCameraInfo(float cx,float cy,float foc);
		void setCameraToGround(float cameraHeight);

		void resetColorMap();
		void saveColorMap(const char* name);
		void loadColorMap(const char* name);
		bool drawSegments;
		float heightCoef;
		int colorMapUsed;

		int classifyPixel(Vec3b a);
		int classifySegment(SSegment s);
		void rgbToHsv(unsigned char r, unsigned char  g, unsigned char b, unsigned int *h, unsigned char *s, unsigned char *v );

		Mat *hsv;
		unsigned char colorArray[64*64*64];
		SSegment segmentArray[MAX_SEGMENTS];
		SSegment bigSegmentArray[MAX_SEGMENTS];
		bool debug;
		int numSegments;
		int numBigSegments;
		float minConvexity,minCircularity;
		float ht[4];
		float st[4];
		float vt[4];
		int width, height;
		float camCX,camCY,focal;
		float cameraToGround;
};

#endif

/* end of CSegmentation.h */
