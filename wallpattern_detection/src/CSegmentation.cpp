#include "CSegmentation.h"

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

int compareSegments(const void* m1,const void* m2)
{
	if (((SSegment*)m1)->size >  ((SSegment*)m2)->size) return -1;
	if (((SSegment*)m1)->size <  ((SSegment*)m2)->size) return 1;
	return 0;
}

//Vymazani promennych a jejich nastaveni na pocatecni hodnoty
CSegmentation::CSegmentation()
{
	memset(colorArray,0,64*64*64);
	debug = true;
	drawSegments = true;

	camCX=320;
	camCY=240;
	focal=320;
	cameraToGround = 0.91;
	float hta[] = {0, 50, 12,193, 15};
	float sta[] = {0, 54,202,142,212};
	float vta[] = {0,178,184,  6,219};
	memcpy(ht,hta,5*sizeof(float));
	memcpy(st,sta,5*sizeof(float));
	memcpy(vt,vta,5*sizeof(float));
}

CSegmentation::~CSegmentation()
{
}

void CSegmentation::saveColorMap(const char* name)
{
	FILE* f=fopen(name,"w");
	fwrite(colorArray,64*64*64,1,f); 
	fclose(f);
}

void CSegmentation::loadColorMap(const char* name)
{
	int stuff = 0;
	FILE* f=fopen(name,"r");
	if (f == NULL) return;
	stuff = fread(colorArray,64*64*64,1,f); 
	fclose(f);
}

int CSegmentation::loadColors(const char* name)
{
	FILE* file = fopen(name,"r");
	if (file == NULL) return -1;
	int index = 0;
	float h,s,v;
	for (int i=0;i<5;i++){
		fscanf(file,"%i %f %f %f\n",&index,&h,&s,&v);
		setColor(index,h,s,v);	
	}
	fclose(file);
	return 0;
}

//smaze indexovaci tabulku
void CSegmentation::resetColorMap()
{
	memset(colorArray,0,64*64*64);
}

void CSegmentation::learnPixel(Vec3b a,int type)
{
	int b = ((a[0]/4)*64+a[1]/4)*64+a[2]/4;
	colorArray[b] = type;
	int addr = 0;
	for (int r=0;r<256;r+=4){
		for (int g=0;g<256;g+=4){
			for (int b=0;b<256;b+=4){
				addr++;
				//colorArray[i] = evaluatePixel3(u);
				//if (classifyPixel(u) > 0) colorArray[i] = type;
			}
		}
	}
}

//nauci se dany pixel
void CSegmentation::learnPixel(unsigned char* learned,int type)
{
	//prevede nauceny pixel do HSV
	unsigned int learnedHue;
	unsigned char learnedSaturation;
	unsigned char learnedValue;
	rgbToHsv(learned[0],learned[1],learned[2],&learnedHue,&learnedSaturation,&learnedValue);

	//z daneho vzoru vytvori indexovaci tabulku
	unsigned char u[3];
	for (int r=0;r<256;r+=4){
		u[0] = r;
		for (int g=0;g<256;g+=4){
			u[1] = g;
			for (int b=0;b<256;b+=4){
				u[2] = b;
				int i = ((r/4)*64+g/4)*64+b/4;
				//colorArray[i] = evaluatePixel3(u);
				//if (classifyPixel(u) > 0) colorArray[i] = type;
			}
		}
	}
	fprintf(stdout,"Learned RGB: %i %i %i, HSV: %i %i %i\n",learned[0],learned[1],learned[2],learnedHue,learnedSaturation,learnedValue);
}

//vrati podobnost pixelu metodou indexovaci tabulky - metoda z dilu IV
int CSegmentation::classifyPixel(Vec3b a)
{
	int b = ((a[0]/4)*64+a[1]/4)*64+a[2]/4;
	return colorArray[b];
}

SSegment CSegmentation::getSegment(int type,int number)
{
	SSegment result;
	result.x = segmentArray[number].x;
	result.y = segmentArray[number].y;
	return result;
}

void CSegmentation::learnPixel(int minHue,int maxHue,int minSat,int maxSat,int minVal,int maxVal,int type)
{
	printf("%i %i %i %i %i %i\n", minHue,maxHue,minSat,maxSat,minVal,maxVal);
	Mat hsv = Mat(1, 1,CV_8UC3);
	Mat rgb = Mat(1, 1,CV_8UC3);
	if (minHue < 0) minHue = 0;
	if (minSat < 0) minSat = 0;
	if (minVal < 0) minVal = 0;
	if (maxHue > 179) maxHue = 179;
	if (maxSat > 255) maxSat = 255;
	if (maxVal > 255) maxVal = 255;
	for (int r=0;r<64;r++){
		for (int g=0;g<64;g++){
			for (int b=0;b<64;b++){
				Vec3b v(r*4,g*4,b*4);
				rgb.at<Vec3b>(0,0) = v;
				cvtColor(rgb,hsv,COLOR_RGB2HSV);
				Vec3b a = hsv.at<Vec3b>(0,0);
				if (a(0) >= minHue && a(0) <= maxHue && a(1) >= minSat && a(1) <= maxSat && a(2) >= minVal && a(2) <=maxVal)
				{
					int i = (r*64+g)*64+b;
					colorArray[i] = type;
				}
			}
		}
	}


	for (int hue=minHue;hue<=maxHue;hue++){
		for (int sat=minSat;sat<=maxSat;sat++){
			for (int val=minVal;val<=maxVal;val++){
				Vec3b b( hue, sat, val );
				hsv.at<Vec3b>(0,0) = b;
				//cout << "HSV: " << b << endl;
				cvtColor(hsv,rgb, COLOR_HSV2RGB);
				Vec3b a = rgb.at<Vec3b>(0,0);
				//cout << "RGB: " << a << endl;
				int i = ((a[0]/4)*64+a[1]/4)*64+a[2]/4;
				colorArray[i] = type;
			}
		}
	}
}


SSegment CSegmentation::separateContours(int *inBuffer,Mat *coords,SSegment *output,int minSize,int maxSize)
{
	static int run = 0;
	SSegment result;
	result.x = -1;
	result.y = -1;

	int expand[8] = {width,-width,1,-1,1+width,-1+width,1-width,-1-width};
	int* stack = (int*)calloc(width*height,sizeof(int));
	int* contour = (int*)calloc(width*height,sizeof(int));
	int stackPosition = 0;
	int contourPoints = 0;

	int type =0;
	numSegments = 0;
	int len = width*height;
	int *buffer = (int*)calloc(width*height,sizeof(int));

	//oznacime oblasti s hledanou barvou
	for (int i = 0;i<len;i++) buffer[i] = -(inBuffer[i]>1000000);
	int borderType = 1000;

	//'ukrojime' okraje obrazu
	int topPos =  0;
	int bottomPos =  height-1;
	int leftPos =  0;
	int rightPos =  width-1;

	for (int i = leftPos;i<rightPos;i++){
		buffer[topPos*width+i] = borderType;	
		buffer[bottomPos*width+i] =  borderType;
	}

	for (int i = topPos;i<bottomPos;i++){
		buffer[width*i+leftPos] =  borderType;	
		buffer[width*i+rightPos] =  borderType;
	}

	int pos = 0;
	//zacneme prohledavani
	int position = 0;
	int queueStart = 0;
	int queueEnd = 0;
	int nncount = 0;
	for (int i = 0;i<len;i++){
		//pokud je nalezen pixel s hledanou barvou, 
		if (buffer[i] < 0 && numSegments < MAX_SEGMENTS){
			queueStart = 0;
			queueEnd = 0;
			contourPoints = 0;
			//zalozime dalsi segment
			segmentArray[numSegments].type = -buffer[i]; 
			type = buffer[i]; 
			buffer[i] = ++numSegments;
			segmentArray[numSegments-1].id = numSegments;
			segmentArray[numSegments-1].size = 1; 
			segmentArray[numSegments-1].warning = 0; 
			segmentArray[numSegments-1].x = i%width; 
			segmentArray[numSegments-1].y = i/width; 
			//a umistime souradnice pixelu na vrchol zasobniku
			stack[queueEnd++] = i;
			//dokud neni zasobnik prazdny
			while (queueEnd != queueStart){
				//vyjmeme ze zasobniku pozici posledne vlozeneho pixelu 
				position = stack[queueStart++];
				nncount = 0;
				//prohledame pixely na sousednich pozicich
				for (int j =0;j<8;j++){
					pos = position+expand[j];
					//a pokud maji hledanou barvu,
					if (buffer[pos] != 0) nncount++;
					if (buffer[pos] == type){
						//pridame jejich pozici do souradnic aktualniho segmentu
						stack[queueEnd++] = pos;
						//a otagujem je 
						buffer[pos] = numSegments;
					}
					if (buffer[pos] == borderType) segmentArray[numSegments-1].warning = 1; 
				}
			}
			if (queueEnd > minSize && queueEnd < maxSize)
			{
				if (segmentArray[numSegments-1].warning == 0){
					long long int sx,sy;
					sx=sy=0;
					for (int s = 0;s<contourPoints;s++){
						pos = contour[s];
						buffer[pos] = 1000000+numSegments;	
					}
					//bounding box + mean RGB + COG
					for (int s = 0;s<queueEnd;s++){
						pos = stack[s];
						sx += pos%width;
						sy += pos/width;
					}
					float fsx = (float)sx/queueEnd; 
					float fsy = (float)sy/queueEnd;

					segmentArray[numSegments-1].size = queueEnd; 
					segmentArray[numSegments-1].x = fsx;
					segmentArray[numSegments-1].y = fsy;

					segmentArray[numSegments-1].px = (segmentArray[numSegments-1].x-camCX)/focal*cameraToGround;
					segmentArray[numSegments-1].py = (segmentArray[numSegments-1].y-camCY)/focal*cameraToGround;

					printf("Contour %i %i %i %f %f\n",run,segmentArray[numSegments-1].warning,queueEnd,segmentArray[numSegments-1].px,segmentArray[numSegments-1].py);
				}

			}else{
				numSegments--;
			}
		}
	}
	run++;
	free(buffer);
	free(stack);
	free(contour);
	return result;
}

void CSegmentation::setCameraInfo(float cx,float cy,float foc)
{
	camCX = cx;
	camCY = cy;
	focal = foc;
}

//segmentace obrazu - metoda z dilu IV
SSegment CSegmentation::findSegment(Mat *image,Mat *coords,SSegment *output,int minSize,int maxSize)
{
	SSegment result;
	result.x = -1;
	result.y = -1;
	width = image->cols;
	height = image->rows;

	int expand[4] = {width,-width,1,-1};
	int* stack = (int*)calloc(width*height,sizeof(int));
	int* contour = (int*)calloc(width*height,sizeof(int));
	int stackPosition = 0;
	int contourPoints = 0;

	int type =0;
	numSegments = 0;
	int len = width*height;
	int *buffer = (int*)calloc(width*height,sizeof(int));

	//oznacime oblasti s hledanou barvou
	for (int i = 0;i<len;i++) buffer[i] = -classifyPixel(image->at<Vec3b>(i/width,i%width));
	int borderType = 1000;

	//'ukrojime' okraje obrazu
	int topPos =  0;
	int bottomPos =  height-1;
	int leftPos =  0;
	int rightPos =  width-1;

	for (int i = leftPos;i<rightPos;i++){
		buffer[topPos*width+i] = borderType;	
		buffer[bottomPos*width+i] =  borderType;
	}

	for (int i = topPos;i<bottomPos;i++){
		buffer[width*i+leftPos] =  borderType;	
		buffer[width*i+rightPos] =  borderType;
	}

	int pos = 0;
	//zacneme prohledavani
	int position = 0;
	int queueStart = 0;
	int queueEnd = 0;
	int nncount = 0;
	for (int i = 0;i<len;i++){
		//pokud je nalezen pixel s hledanou barvou, 
		if (buffer[i] < 0 && numSegments < MAX_SEGMENTS){
			queueStart = 0;
			queueEnd = 0;
			contourPoints = 0;
			//zalozime dalsi segment
			segmentArray[numSegments].type = -buffer[i]; 
			type = buffer[i]; 
			buffer[i] = ++numSegments;
			segmentArray[numSegments-1].id = numSegments;
			segmentArray[numSegments-1].size = 1; 
			segmentArray[numSegments-1].warning = 0; 
			segmentArray[numSegments-1].x = i%width; 
			segmentArray[numSegments-1].y = i/width; 
			//a umistime souradnice pixelu na vrchol zasobniku
			stack[queueEnd++] = i;
			//dokud neni zasobnik prazdny
			while (queueEnd != queueStart){
				//vyjmeme ze zasobniku pozici posledne vlozeneho pixelu 
				position = stack[queueStart++];
				nncount = 0;
				//prohledame pixely na sousednich pozicich
				for (int j =0;j<4;j++){
					pos = position+expand[j];
					//a pokud maji hledanou barvu,
					if (buffer[pos] != 0) nncount++;
					if (buffer[pos] == type){
						//pridame jejich pozici do souradnic aktualniho segmentu
						stack[queueEnd++] = pos;
						//a otagujem je 
						buffer[pos] = numSegments;
					}
					if (buffer[pos] == borderType) segmentArray[numSegments-1].warning = 1; 
				}
				//is this a border point?
				if (nncount != 4) contour[contourPoints++] = position;
			}
			if (queueEnd > minSize && queueEnd < maxSize){
				long long int cx,cy,sx,sy,sh,ss,sv;
				long long int chh,chs,chv,css,csv,cvv;
				long long int cxx,cxy,cyy; 
				int maxX,maxY,minX,minY;
				int ch,cs,cv;
				maxX=maxY= -1;
				minX=minY = width*height;
				cxx=cxy=cyy=chh=chs=chv=css=csv=cvv=sx=sy=sh=ss=sv=0;
				for (int s = 0;s<contourPoints;s++){
					pos = contour[s];
					buffer[pos] = 1000000+numSegments;	
				}
				//bounding box + mean RGB + COG
				for (int s = 0;s<queueEnd;s++){
					pos = stack[s];
					cx = pos%width; 
					cy = pos/width;
					Vec3b vv = image->at<Vec3b>(cy,cx);
					ch = vv[0];	
					cs = vv[1];	
					cv = vv[2];	
					sh += ch;
					ss += cs;
					sv += cv;
				        chh += ch*ch;	
				        chs += ch*cs;	
				        chv += ch*cv;	
				        css += cs*cs;	
				        csv += cs*cv;	
				        cvv += cv*cv;
				        sx += cx;	
				        sy += cy;
					cxx += cx*cx; 
					cxy += cx*cy; 
					cyy += cy*cy; 
					if (minX > cx) minX = cx;
					if (minY > cy) minY = cy;
					if (maxX < cx) maxX = cx;
					if (maxY < cy) maxY = cy;
				}
				float fsx = (float)sx/queueEnd; 
				float fsy = (float)sy/queueEnd;
				float fsh = (float)sh/queueEnd;
				float fss = (float)ss/queueEnd;
				float fsv = (float)sv/queueEnd;
				unsigned char ur = fsh;  
				unsigned char ug = fss;  
				unsigned char ub = fsv;
				unsigned int uh;
				unsigned char us,uv;
				rgbToHsv(ur,ug,ub,&uh,&us,&uv);
				fsh = uh;
				fss = us;
				fsv = uv;
				float fchh = ((float)chh/queueEnd-fsh*fsh);
				float fcss = ((float)css/queueEnd-fss*fss);
				float fcvv = ((float)cvv/queueEnd-fsv*fsv);
				float fcxx = ((float)cxx/queueEnd-fsx*fsx);
				float fcxy = ((float)cxy/queueEnd-fsy*fsx);
				float fcyy = ((float)cyy/queueEnd-fsy*fsy);
				float det = (fcxx+fcyy)*(fcxx+fcyy)-4*(fcxx*fcyy-fcxy*fcxy);
				if (det > 0) det = sqrt(det); else det = 0;
				float eigvl0 = sqrt(((fcxx+fcyy)+det)/2);
				float eigvl1 = sqrt(((fcxx+fcyy)-det)/2);

				if (fcyy != 0){                                                            
					segmentArray[numSegments-1].v0 = -fcxy/sqrt(fcxy*fcxy+(fcxx-eigvl0)*(fcxx-eigvl0));
					segmentArray[numSegments-1].v1 = (fcxx-eigvl0)/sqrt(fcxy*fcxy+(fcxx-eigvl0)*(fcxx-eigvl0));
				}else{
					segmentArray[numSegments-1].v0 = segmentArray[numSegments-1].v1 = 0;
					if (fcxx > fcyy) segmentArray[numSegments-1].v0 = 1.0; else segmentArray[numSegments-1].v1 = 1.0;
				}
				segmentArray[numSegments-1].m0 = eigvl0; 
				segmentArray[numSegments-1].m1 = eigvl1;
                                //std::cout << queueEnd<< " cxx " << cxx<<" cxy "<<cxy<<" cyy " << cyy<<" fcxx " <<fcxx<<" fcyy "<< fcyy<< " fcxy "<<fcxy<< " det "<<det<<" type "<<segmentArray[numSegments].type<<std::endl;
				segmentArray[numSegments-1].size = queueEnd; 
				segmentArray[numSegments-1].x = fsx;
				segmentArray[numSegments-1].y = fsy;
				segmentArray[numSegments-1].h = fsh; 
				segmentArray[numSegments-1].s = fss; 
				segmentArray[numSegments-1].v = fsv; 
				segmentArray[numSegments-1].ch = fchh; 
				segmentArray[numSegments-1].cs = fcss; 
				segmentArray[numSegments-1].cv = fcvv; 
				segmentArray[numSegments-1].minX = minX; 
				segmentArray[numSegments-1].minY = minY; 
				segmentArray[numSegments-1].maxX = maxX; 
				segmentArray[numSegments-1].maxY = maxY; 
				segmentArray[numSegments-1].roundness = M_PI*4*eigvl1*eigvl0/queueEnd;
				segmentArray[numSegments-1].circularity = eigvl1/eigvl0;

				/*calculate and puclish corner candidates*/
				int corners = 0;
				int cX[4];
				int cY[4];
				float dist,maxDist;
				for (int cn = 0;cn<4;cn++){
					maxDist = 0;
					for (int s = 0;s<contourPoints;s++)
					{
						pos = contour[s];
						cx = pos%width-fsx; 
						cy = pos/width-fsy;
						dist = 0;
						if (cn > 0)
						{
							for (int c = 0;c<cn;c++) dist+=sqrt((cx-cX[c])*(cx-cX[c])+(cy-cY[c])*(cy-cY[c]));
						}else{
							dist = cx*cx+cy*cy;
							if (s < MAX_CONTOUR_POINTS){
								segmentArray[numSegments-1].contourX[s] = cx+fsx-minX;
								segmentArray[numSegments-1].contourY[s] = cy+fsy-minY;
							}
						}
						if (dist > maxDist)
						{
							cX[cn] = cx;	
							cY[cn] = cy;
							maxDist = dist;
						}
					}
				}
				segmentArray[numSegments-1].contourPoints = min(contourPoints,MAX_CONTOUR_POINTS);
				segmentArray[numSegments-1].combo = 1; 
				for (int ii = 0;ii<4;ii++){
					segmentArray[numSegments-1].cornerX[ii] = cX[ii]+fsx;//-minX;
					segmentArray[numSegments-1].cornerY[ii] = cY[ii]+fsy;//-minY;
				}

                                //std::cout << "S "<<numSegments << "qEnd "<<queueEnd<<" fsx "<<fsx<<" cx "<<segmentArray[numSegments-1].x<<" fsy "<<fsy<<" cy "<<segmentArray[numSegments-1].y<< " seg size "<<(maxX-minX)<<","<<(maxY-minY)<< "combo " << segmentArray[numSegments-1].combo << std::endl;
				//if (peak > 0 && peak > queueEnd/100 && peak < queueEnd/10 && fabs(fsx-segmentArray[numSegments-1].x) < 5 && fabs(fsy-segmentArray[numSegments-1].y) < 5) segmentArray[numSegments-1].combo = 1; else segmentArray[numSegments-1].combo = 0;
				//segmentArray[numSegments-1].combo = peak;
				//if (segmentArray[numSegments-1].roundness >  1.0) segmentArray[numSegments-1].roundness  = 1.0/segmentArray[numSegments-1].roundness;
				/*real 'roundness' */
				//if (segmentArray[numSegments-1].roundness > minCircularity) segmentArray[numSegments-1].roundness = 4*sqrt(queueEnd)/contourPoints;
			}else{
				numSegments--;
			}
		}
	}

	//Seradi segmenty podle velikosti
	qsort(segmentArray,numSegments,sizeof(SSegment),compareSegments);
	//separateContours(buffer,coords,output,10,1000000);
	if (false){
		*coords = cv::Mat::ones(0, 1, CV_32FC2);
		Mat coord = cv::Mat::ones(1, 1, CV_32FC2);
		int taken = -1;

		//A vyhodi je do vystupni matice
		for (int i = 0;i<numSegments;i++){
			if (segmentArray[i].roundness > minCircularity && segmentArray[i].combo > 0 && segmentArray[i].warning == 0)
			{

				if (taken == - 1) taken = segmentArray[i].id;
				coord.at<float>(0) = segmentArray[i].x;
				coord.at<float>(1) = segmentArray[i].y;
			}
		}
	}
	result = segmentArray[0];
	int i = 0;
	printf("Segment %i %i %f %f\n",i,segmentArray[i].size,segmentArray[i].x,segmentArray[i].y);	
	//vykreslime vysledek
	int j = 0;
	if (drawSegments){
		for (int i = 0;i<len;i++){
			j = buffer[i];
			if (j > 1000000) image->at<Vec3b>(i/width,i%width) = Vec3f(0,0,0);// else image->at<Vec3b>(i/width,i%width) = Vec3f(255,255,255);
		}
	}	
	free(buffer);
	free(stack);
	free(contour);
	return result;
}



void CSegmentation::setColor(int i,float h,float s,float v)
{
	if (i >=0 & i<5){ 	
		ht[i] = h;
		st[i] = s;
		vt[i] = v;
	}
}

int CSegmentation::classifySegment(SSegment s)
{
	int mindex = s.type;
	if (mindex == -1){
		float minimal = 1000000;
		for (int i = 1;i<5;i++){
			float dh = ht[i]-s.h; 
			float ds = st[i]-s.s; 
			float dv = vt[i]-s.v; 
			float mina = sqrt(dh*dh);//(+ds*ds+dv*dv);
			//printf("Clas: %.3f  %.3f %.3f %.3f %.3f %.3f %.3f\n",s.h,s.s,s.v,ht[i],st[i],vt[i],mina);
			if (mina <minimal)
			{
				minimal=mina;
				mindex=i;
			}
		}
	}
	if (mindex == 4)mindex = 5;
	return mindex;
}

//prevod RGB -> HSV, prevzato z www
void CSegmentation::rgbToHsv(unsigned char r, unsigned char  g, unsigned char b, unsigned int *hue, unsigned char *saturation, unsigned char *value )
{
	float min, max, delta;
	float h,s,v;   

	h=s=v=0; 
	*saturation = (unsigned char) s;
	*value = (unsigned char) v;
	*hue = (unsigned int) h;

	min = min( r, min(g, b) );
	max = max( r, max(g, b) );
	v = max;			

	delta = max - min;

	if( max != 0 )
		s = min(delta*255 / max,255);	
	else {
		s = 0;
		h = -1;
		return;
	}

	if( r == max )
		h = ( g - b ) / delta;		// between yellow & magenta
	else if( g == max )
		h = 2 + ( b - r ) / delta;	// between cyan & yellow
	else
		h = 4 + ( r - g ) / delta;	// between magenta & cyan
	h = h*60;
	if (h<0) h+=360;
	*saturation = (unsigned char) s;
	*value = (unsigned char) v;
	*hue = (unsigned int) h;
}

