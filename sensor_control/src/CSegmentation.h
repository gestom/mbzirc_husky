/*
 * File name: CSegmentation.h
 * Date:      2010
 * Author:   Tom Krajnik 
 */

#ifndef __CSEGMENATION_H__
#define __CSEGMENATION_H__

#include "CRawDepthImage.h"
#include <math.h>
#include <stdio.h>
#define MAX_SEGMENTS 1000
#define MIN_SEGMENT_SIZE 40 
#define MAX_CONTOUR_POINTS 1000

using namespace std;


typedef struct{
	float x,y,z;
	float v0,v1;
	float m0,m1;
	int minX,minY,maxX,maxY;
	float ch,cs,cv;
	int cornerX[4];
	int cornerY[4];
	int contourX[MAX_CONTOUR_POINTS];
	int contourY[MAX_CONTOUR_POINTS];
	int contourPoints;
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
		SSegment findSegment(CRawDepthImage* image,int minSegmentSize,int maxSegmentSize);
		SSegment getSegment(int type,int number);

		SSegment segmentArray[MAX_SEGMENTS];
		bool debug;
		bool drawSegments;
		int numSegments,threshold;
};

#endif

/* end of CSegmentation.h */
