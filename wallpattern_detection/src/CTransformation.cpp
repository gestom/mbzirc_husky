#include "CTransformation.h"
#include <stdio.h>
#include "sysmat.h" 

int sortByDistance(const void* m1,const void* m2)
{
        if (((STrackedObject*)m1)->d > ((STrackedObject*)m2)->d) return -1;
        if (((STrackedObject*)m1)->d < ((STrackedObject*)m2)->d) return 1;
        return 0;
}

CTransformation::CTransformation(int widthi,int heighti,float diam,bool fullUnbarreli)
{
	width = widthi;
	height = heighti;
}

CTransformation::~CTransformation()
{
}

STrackedObject CTransformation::transform2D(STrackedObject o)
{
	STrackedObject r;
	r.x = hom[0]*o.x+hom[1]*o.y+hom[2]; 
	r.y = hom[3]*o.x+hom[4]*o.y+hom[5];
	r.z = hom[6]*o.x+hom[7]*o.y+hom[8];
	r.x = r.x/r.z;
	r.y = r.y/r.z;
	r.z = 0;
	return r; 	
}


STrackedObject CTransformation::transform2D(SSegment o)
{
	STrackedObject r;
	r.x = hom[0]*o.x+hom[1]*o.y+hom[2]; 
	r.y = hom[3]*o.x+hom[4]*o.y+hom[5];
	r.z = hom[6]*o.x+hom[7]*o.y+hom[8];
	r.x = r.x/r.z;
	r.y = r.y/r.z;
	r.z = 0;
	r.numContours = o.contours;
	r.validity = o.valid;
	return r; 	
}

void CTransformation::loadCalibration(const char *str)
{
	FILE* file = fopen(str,"r+");
	int k = 0;
	for (int i = 0;i<9;i++){
		if (fscanf(file,"%f ",&hom[i])!=1)fprintf(stderr,"ERROR LOADING CALIBRATION FILE");
		if (i%3 == 2){
			if (fscanf(file,"\n")!=0) fprintf(stderr,"ERROR LOADING CALIBRATION FILE");
		}
	}
	fclose(file);
}

void CTransformation::saveCalibration(const char *str)
{
	FILE* file = fopen(str,"w+");
	fprintf(file,"2D_calibration\n");
	for (int i = 0;i<9;i++){
		fprintf(file,"%f ",hom[i]);
		if (i%3 == 2) fprintf(file,"\n");
	}
	fclose(file);
}

int CTransformation::calibrate2D(const char *fileName)
{
	STrackedObject o[4],r[4];
	FILE *pointFile = fopen(fileName,"r");
	for (int i = 0;i<4;i++) fscanf(pointFile,"%f %f %f %f\n",&o[i].x,&o[i].y,&r[i].x,&r[i].y);	
	fclose(pointFile);

	MAT est;
	MAT1 vec;
	REAL det;
	for (int i = 0;i<4;i++){
		est[2*i][0]=-o[i].x;
		est[2*i][1]=-o[i].y;
		est[2*i][2]=-1;
		est[2*i][3]=0;
		est[2*i][4]=0;
		est[2*i][5]=0;
		est[2*i][6]=r[i].x*o[i].x;
		est[2*i][7]=r[i].x*o[i].y;
		est[2*i+1][0]=0;
		est[2*i+1][1]=0;
		est[2*i+1][2]=0;
		est[2*i+1][3]=-o[i].x;
		est[2*i+1][4]=-o[i].y;
		est[2*i+1][5]=-1;
		est[2*i+1][6]=r[i].y*o[i].x;
		est[2*i+1][7]=r[i].y*o[i].y;
		vec[2*i][0]=-r[i].x;
		vec[2*i+1][0]=-r[i].y;
	}
	MATINV(8,1,est,vec,&det); 
	for (int i = 0;i<8;i++)  hom[i] = vec[i][0];
	hom[8] = 1;
	for (int i = 0;i<4;i++){
		r[i] = transform2D(o[i]);
		printf("%.2f %.2f %.2f %.2f\n",o[i].x,o[i].y,r[i].x,r[i].y);
	}
	return 0;
}
