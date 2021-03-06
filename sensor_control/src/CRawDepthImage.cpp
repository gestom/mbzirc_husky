#include "CRawDepthImage.h"

static unsigned char sth[] = {66,77,54,16,14,0,0,0,0,0,54,0,0,0,40,0,0,0,128,2,0,0,224,1,0,0,1,0,24,0,0,0,0,0,0,16,14,0,18,11,0,0,18,11,0,0,0,0,0,0,0,0,0,0};

CRawDepthImage::CRawDepthImage(int wi,int he,int bppi)
{
	width =  wi;
	height = he;
	bpp= bppi;
	size = width*height;
	data = (short*)calloc(size,sizeof(short));
	memset(header,0,122);
	memcpy(header,sth,122);
	header[18] = width%256;
	header[19] = width/256;
	header[22] = height%256;
	header[23] = height/256;
	header[2] = (size+122)%256;
	header[3] = ((size+122)/256)%256;
	header[4] = ((size+122)/256/256)%256;
	header[5] = ((size+122)/256/256)/256;
	header[34] = (size)%256;
	header[35] = ((size)/256)%256;
	header[36] = ((size)/256/256)%256;
	header[37] = ((size)/256/256)/256;
	header[10] = 122;
	numSaved = 0;
	ownData = true;
	heightLimit = height;
}

CRawDepthImage::CRawDepthImage(short *datai,int wi,int he,int bppi)
{
	ownData = false;
	width =  wi;
	height = he;
	bpp= bppi;
	size = width*height;
	data = datai; 
	memset(header,0,122);
	memcpy(header,sth,122);
	header[18] = width%256;
	header[19] = width/256;
	header[22] = height%256;
	header[23] = height/256;
	header[2] = (size+54)%256;
	header[3] = ((size+54)/256)%256;
	header[4] = ((size+54)/256/256)%256;
	header[5] = ((size+54)/256/256)/256;
	header[34] = (size)%256;
	header[35] = ((size)/256)%256;
	header[36] = ((size)/256/256)%256;
	header[37] = ((size)/256/256)/256;
	header[10] = 122;
	numSaved = 0;
}

int CRawDepthImage::getClosest(int groundPlaneDistance)
{
	int granularity = 100;
	int histogram[50];
	int dat;
	int ground = 0;
	int maxValue = 0;
	int maxIndex = -1;
	if (groundPlaneDistance == 0){
		/*estimate the ground plane*/
		for (int i=0;i<50;i++) histogram[i] = 0;
		for (int i=0;i<size;i++){
			dat = data[i];
			if (dat > 0 && dat<50*100) histogram[data[i]/100]++;
		}
		maxValue = 0;
		maxIndex = -1;

		for (int i=0;i<50;i++){
			if (maxValue < histogram[i]){
				maxValue = histogram[i];
				maxIndex = i;
			}
			//printf("Histogram %i %i\n",i,histogram[i]);
		}
		int sumGround = 0;
		int numGround = 0;
		for (int i=0;i<size;i++)
		{
			if (data[i] < (maxIndex+1)*granularity && data[i] >  (maxIndex -1)*granularity)
			{
				sumGround += data[i];
				numGround++;		
			} 
		}
		ground = sumGround/numGround;
	}else{
		ground = groundPlaneDistance;
	}

	/*recalculate histogram*/
	int brickSize = 200;
	int brickTolerance = 50;

	for (int i=0;i<50;i++) histogram[i] = 0;
	for (int i=0;i<size;i++){
		dat = ground-data[i];
		if (dat > 0 && dat<50*100){
		       	histogram[dat/100]++;
		}
	}
	
	/*segment out a brick*/
	for (int i=0;i<size;i++)
	{
		dat = ground-data[i];
		if (dat - 2*brickSize < brickTolerance && dat-2*brickSize > -brickTolerance) continue;
		if (dat - 1*brickSize < brickTolerance && dat-1*brickSize > -brickTolerance) continue;
	       	data[i] = 0;
	}
	//for (int i=size/width*heightLimit;i<size;i++) data[i] = 0;
}

int CRawDepthImage::getSaveNumber()
{
	char name[100];
	FILE* file = NULL;
	do{
		sprintf(name,"%04i.bmp",numSaved++);
		file = fopen(name,"r");
	}
	while (file != NULL);
	numSaved--;
	return numSaved;
}

CRawDepthImage::~CRawDepthImage()
{
	if (ownData) free(data);
}

void CRawDepthImage::swap()
{
}

void CRawDepthImage::saveBmp(const char* inName)
{
	FILE* file = fopen(inName,"wb");
	swap();
	fwrite(header,54,1,file);
	fwrite(header,54,1,file);
	fwrite(header,14,1,file);
	unsigned char *saveData = (unsigned char*)calloc(size*3,sizeof(unsigned char));
	for (int i = 0;i<size;i++){
		saveData[3*i+0]=saveData[3*i+1]=saveData[3*i+2] = data[i]/10;	
		if (data[i] < 0){
			saveData[3*i+0]=255;	
			saveData[3*i+1]=0;	
			saveData[3*i+1]=0;	
		}
	}
	fwrite(saveData,size*3,1,file);
	free(saveData);
	printf("Saved size %ix%i - %i.\n",size,width,height);
	fclose(file);
}

void CRawDepthImage::generateRGB(unsigned char* saveData)
{
	for (int i = 0;i<size;i++){
		saveData[3*i+0]=saveData[3*i+1]=saveData[3*i+2] = data[i]/10;	
		if (data[i] < 0 && data[i] > -4){
			saveData[3*i+0]= saveData[3*i+1]=saveData[3*i+1]=0;
			saveData[3*i-data[i]-1] = 255; 
		}
	}
}


void CRawDepthImage::saveBmp()
{
	char name[100];
	sprintf(name,"images/%06i.bmp",numSaved++);
	saveBmp(name);
}

bool CRawDepthImage::loadBmp(const char* inName)
{
/*	FILE* file = fopen(inName,"rb");
	if (file!=NULL)
	{
		if (fread(data,54,1,file)!=1) fprintf(stderr,"Warning! Image header could not be read.\n");;
		bpp = 3;
		memcpy(header,data,54);
		int headerWidth = header[18]+header[19]*256;
		int headerHeight = header[22]+header[23]*256;
		if (ownData && (headerWidth != width || headerHeight != height)){
			free(data);
			height = headerHeight;
			width = headerWidth;
			size = height*width*bpp;
			data = (unsigned char*)calloc(size,sizeof(unsigned char));
		}
		int offset = header[10]+header[11]*256;
		if (offset-54 > 0 && fread(data,offset-54,1,file)!=1) fprintf(stderr,"Warning! Image header could not be read.\n");;
		if (fread(data,size,1,file)!=1) fprintf(stderr,"Warning! Image data could not be read.\n");;
		fclose(file);
		swap();
		return true;
	}*/
	return false;
}

void CRawDepthImage::plotCenter()
{
	int centerWidth = 20;
	unsigned char color[] = {255,150,150};
	for (int i = -centerWidth;i<centerWidth;i++){
		for (int j =0;j<3;j++){
			data[(width*(height/2+i)+width/2-centerWidth)*3+j] = color[j];
			data[(width*(height/2+i)+width/2+centerWidth)*3+j] = color[j];
			data[(width*(height/2-centerWidth)+width/2+i)*3+j] = color[j];
			data[(width*(height/2+centerWidth)+width/2+i)*3+j] = color[j];
		}
	}
}

void CRawDepthImage::plotLine(int x,int y) {
	int base;
	if (y < 0 || y > height-1) y = height/2;
	if (x < 0 || x > width-1) x = width/2;
	for(int i=0; i < width;i++) {
		if (i == width/2) i++;
		base = (width*y+i)*3;
		data[base+0] = 255;
		data[base+1] = 0;
		data[base+2] = 255;
	}

	for(int j=0;j<height;j++) {
		const int bidx = ((width*j)+x)*3;
		if (j == height/2) j++;
		data[bidx+0] = 255;
		data[bidx+1] = 255;
		data[bidx+2] = 0;
	}
}


/** pocita jas obrazku:
  *  upperHalf == true, pocita se jen z horni poloviny obrazku
  *  upperHalf == false, pocita jen ze spodni poloviny obrazku
  */
double CRawDepthImage::getOverallBrightness(bool upperHalf) {
	int step = 5;
	int sum,num,satMax,satMin,pos;
	sum=num=satMax=satMin=0;
	int limit = 0;
	if (upperHalf) limit = 0; else limit=height/2;
	for (int i = limit;i<height/2+limit;i+=step){
		for (int j = 0;j<width;j+=step){
			pos = (i*width+j)*bpp;
			if (data[pos] >= 250 && data[pos+1] >=250 && data[pos+2] >= 250) satMax++;  
			if (data[pos] <= 25 && data[pos+1] <=25 && data[pos+2] <= 25) satMin++;
			sum+=data[pos] + data[pos+1] + data[pos+2];
			num++;
		}
	}
	return (sum/num/bpp) + (satMax-satMin)*100.0/num;
}




