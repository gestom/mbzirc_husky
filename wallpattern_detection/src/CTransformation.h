/*
 * File name: CTransformation.h
 * Date:      2005/11/07 18:10
 * Author:    
 */

#ifndef __CTRANSFORMATION_H__
#define __CTRANSFORMATION_H__

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "CSegmentation.h"

typedef enum{
	TRANSFORM_NONE,
	TRANSFORM_2D,
	TRANSFORM_3D,
	TRANSFORM_4D,
	TRANSFORM_INV,
	TRANSFORM_NUMBER
}ETransformType;

typedef struct{
	float x,y,z,d;
	float pitch,roll,yaw;
	float roundness;
	float bwratio;
	float error;
	float esterror;
	int ID;
}STrackedObject;

class CTransformation
{
	public:
		CTransformation(int widthi = 640,int heighti = 480,float diam=0.08,bool fullUnbarreli = false);
		~CTransformation();

		int calibrate2D(const char* name);

		void saveCalibration(const char *str);
		void loadCalibration(const char *str);

		STrackedObject  normalize(STrackedObject o);
		STrackedObject  transform2D(STrackedObject o);
		STrackedObject transform2D(SSegment o);
		float *xArray;
		float *yArray;
		float *gArrayX;
		float *gArrayY;
		int *pArray;

		float hom[9];
		float gDimX,gDimY;

		int width,height;
};

#endif
/* end of CTransformation.h */
