/*
 * File name: CSegmentation.h
 * Date:      2010
 * Author:   Tom Krajnik 
 */

#ifndef __CSEGMENATION_H__
#define __CSEGMENATION_H__

#include "CRawDepthImage.h"
#include "CRawImage.h"
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
	int cornerX[4];
	int cornerY[4];
	float cPX[4];
	float cPY[4];
	int contourX[MAX_CONTOUR_POINTS];
	int contourY[MAX_CONTOUR_POINTS];
	int contourPoints;
	int id;
	int size;
	float crit;
	int type;
	int warning;
	int valid;
	float angle;
	float ratio1;
	float ratio2;
	float sideRatio;
}SSegment;

class CSegmentation
{
	public:
		CSegmentation();
		~CSegmentation();
		SSegment findSegment(CRawDepthImage* image,int minSegmentSize,int maxSegmentSize,int wantedType = 0,CRawImage* colorImage = NULL);
		SSegment getSegment(int type,int number);
		void resetTracking(CRawDepthImage* im,int iX = 0,int iY = 0);

		SSegment segmentArray[MAX_SEGMENTS];
		bool debug;
		bool drawSegments;
		int numSegments,threshold;
		float sizeRatioTolerance;
		SSegment priorPosition;
		SSegment defaultPosition;
};

#endif

/* end of CSegmentation.h */
