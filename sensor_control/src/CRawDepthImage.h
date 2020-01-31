#ifndef CDEPTHIMAGE_H
#define CDEPTHIMAGE_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//#define THICK_CROSS

/**
@author Tom Krajnik
*/
class CRawDepthImage
{
public:

  CRawDepthImage(int wi,int he,int bppi);
  CRawDepthImage(short *datai,int wi,int he,int bppi);
  ~CRawDepthImage();
  void generateRGB(unsigned char * bla);
  void saveBmp(const char* name);
  void saveBmp();
  bool loadBmp(const char* name);
  void swap();

  void plotLine(int x,int y);
  void plotCenter();

  int getClosest(int number);
  int  getSaveNumber();

  double getOverallBrightness(bool upperHalf);
  
	  
  int width;
  int height;
  int palette;
  int size;
  int bpp;
  unsigned char header[122]; 

  short* data;
  bool ownData;
  int numSaved;
};

#endif
