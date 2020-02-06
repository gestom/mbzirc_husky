#include "CSegmentation.h"

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

int compareSegments(const void* m1,const void* m2)
{
	//if (((SSegment*)m1)->size <  ((SSegment*)m2)->size) return +1;
	//if (((SSegment*)m1)->size >  ((SSegment*)m2)->size) return -1;
	if (((SSegment*)m1)->crit <  ((SSegment*)m2)->crit) return +1;
	if (((SSegment*)m1)->crit >  ((SSegment*)m2)->crit) return -1;
	return 0;
}

//Vymazani promennych a jejich nastaveni na pocatecni hodnoty
CSegmentation::CSegmentation()
{
	debug = true;
	drawSegments = true;
	sizeRatioTolerance=0.25;
	segmentArray[0].valid = false;
}

CSegmentation::~CSegmentation()
{
}

void CSegmentation::resetTracking(CRawDepthImage* im,int iX,int iY)
{
	if (iX == 0) iX = im->width/2; 
	if (iY == 0) iY = im->height/2;
        defaultPosition.x = iX;	
        defaultPosition.y = iY;	
	priorPosition = defaultPosition;
	segmentArray[0].valid = 0;
}

SSegment CSegmentation::getSegment(int type,int number)
{
	SSegment result;
	result.x = segmentArray[number].x;
	result.y = segmentArray[number].y;
	return result;
}

SSegment CSegmentation::findSegment(CRawDepthImage *image,int minSize,int maxSize,int wantedType)
{
	//if (segmentArray[0].valid && numSegments > 0) priorPosition =  segmentArray[0]; else priorPosition = defaultPosition;

	segmentArray[0].valid = -1;
	int width = image->width;
	int height = image->height;

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
	for (int i = 0;i<len;i++) buffer[i] = -image->data[i];
	int borderType = 10000;

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
			segmentArray[numSegments-1].z = image->data[i];
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
					if (buffer[pos] < 0 && fabs(buffer[pos]-type) < 100) nncount++; //TODO indoor/outdoor settings
					//if (buffer[pos] != 0) nncount++;
					if (buffer[pos] < 0 && fabs(buffer[pos]-type) < 100){
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
				long long int cx,cy,cz,sx,sy,sz;
				long long int cxx,cxy,cyy; 
				int maxX,maxY,minX,minY;
				maxX=maxY= -1;
				minX=minY = width*height;
				cxx=cxy=cyy=sx=sy=sz=0;
				for (int s = 0;s<contourPoints;s++){
					pos = contour[s];
					buffer[pos] = 1000000+numSegments;	
				}

				//bounding box
				for (int s = 0;s<queueEnd;s++){
					pos = stack[s];
					cx = pos%width; 
					cy = pos/width;
				        sx += cx;	
				        sy += cy;
					sz += image->data[pos]; 
				}
				
				float fsx = (float)sx/queueEnd; 
				float fsy = (float)sy/queueEnd;
				float fsz = (float)sz/queueEnd;
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
				//printf("%i %f %f %f\n",numSegments-1,fsx,fsy,fsz);
				segmentArray[numSegments-1].x = fsx;
				segmentArray[numSegments-1].y = fsy;
				segmentArray[numSegments-1].z = fsz;
				segmentArray[numSegments-1].minX = minX; 
				segmentArray[numSegments-1].minY = minY; 
				segmentArray[numSegments-1].maxX = maxX; 
				segmentArray[numSegments-1].maxY = maxY; 
				//segmentArray[numSegments-1].roundness = M_PI*4*eigvl1*eigvl0/queueEnd;
				//segmentArray[numSegments-1].circularity = eigvl1/eigvl0;

				/*calculate and publish corner candidates*/
				int corners = 0;
				int cX[4];
				int cY[4];
				int cZ[4];
				float dist;
				float maxDist = 0;

				for (int cn = 0;cn<4;cn++){
					maxDist = 0;
					for (int s = 0;s<contourPoints;s++)
					{
						pos = contour[s];
						cx = pos%width; 
						cy = pos/width;
						//cz = image->data[pos];
						dist = 0;
						if (cn > 0)
						{
							for (int c = 0;c<cn;c++) dist+=sqrt((cx-cX[c])*(cx-cX[c])+(cy-cY[c])*(cy-cY[c]));
						}else{
							dist = sqrt((cx-fsx)*(cx-fsx)+(cy-fsy)*(cy-fsy));
							if (s < MAX_CONTOUR_POINTS){
								segmentArray[numSegments-1].contourX[s] = cx+fsx-minX;
								segmentArray[numSegments-1].contourY[s] = cy+fsy-minY;
							}
						}
						if (dist > maxDist)
						{
							cX[cn] = cx;
							cY[cn] = cy;
							//cZ[cn] = cz;
							maxDist = dist;
						}
					}
				}
				/*ordering points*/
				cx = cX[1];
				cy = cY[1];
				cX[1]=cX[2];
				cY[1]=cY[2];
				cX[2] = cx;
				cY[2] = cy;
				segmentArray[numSegments-1].contourPoints = min(contourPoints,MAX_CONTOUR_POINTS);
				//segmentArray[numSegments-1].combo = 1; 
				segmentArray[numSegments-1].valid = 1; 
				for (int ii = 0;ii<4;ii++){
					segmentArray[numSegments-1].cornerX[ii] = cX[ii];
					segmentArray[numSegments-1].cornerY[ii] = cY[ii];
				}
			}else{
				numSegments--;
			}
		}
	}
	for (int i = 0;i<numSegments;i++)
	{
		float dist[4];
		float dX[4];
		float dY[4];
		float pX,pY;
		float angle;
		for (int ii = 0;ii<4;ii++){
			dX[ii] = segmentArray[i].cornerX[ii]-segmentArray[i].cornerX[(ii+1)%4];
			dY[ii] = segmentArray[i].cornerY[ii]-segmentArray[i].cornerY[(ii+1)%4];
			dist[ii] = sqrt(dX[ii]*dX[ii]+dY[ii]*dY[ii]);
		}
		if (dist[0] > dist[1])
		{
			 pX = (segmentArray[i].cornerX[0]+segmentArray[i].cornerX[3])/2-(segmentArray[i].cornerX[1]+segmentArray[i].cornerX[2])/2;
			 pY = (segmentArray[i].cornerY[0]+segmentArray[i].cornerY[3])/2-(segmentArray[i].cornerY[1]+segmentArray[i].cornerY[2])/2;
			 angle = atan2(pY,pX);
			 if (fabs(dist[1]) > 0.01) segmentArray[i].sideRatio = dist[0]/dist[1];
		}else{
			 pX = (segmentArray[i].cornerX[0]+segmentArray[i].cornerX[1])/2-(segmentArray[i].cornerX[3]+segmentArray[i].cornerX[2])/2;
			 pY = (segmentArray[i].cornerY[0]+segmentArray[i].cornerY[1])/2-(segmentArray[i].cornerY[3]+segmentArray[i].cornerY[2])/2;
			 angle = atan2(pY,pX);
			 if (fabs(dist[0]) > 0.01){
				 segmentArray[i].sideRatio = dist[1]/dist[0];
			 }
		}
		segmentArray[i].type = 1;  
		if (segmentArray[i].sideRatio > 2.3)  segmentArray[i].type = 2; 
		if (segmentArray[i].sideRatio > 4.5)  segmentArray[i].type = 3; 
		if (angle > +M_PI/2) angle = angle - M_PI;
		if (angle < -M_PI/2) angle = angle + M_PI;
		segmentArray[i].angle = angle;
		segmentArray[i].ratio1 = fabs(dist[0]*dist[1]-segmentArray[i].size)/segmentArray[i].size;
		segmentArray[i].ratio2 = fabs(dist[2]*dist[3]-segmentArray[i].size)/segmentArray[i].size;
		if (segmentArray[i].ratio1 >sizeRatioTolerance || segmentArray[i].ratio2 >sizeRatioTolerance) segmentArray[i].valid = 0;
	}


	/*ordering stuff*/
	for (int i = 0;i<numSegments;i++){
	       	segmentArray[i].crit = segmentArray[i].y;
	       	segmentArray[i].crit = segmentArray[i].crit*segmentArray[i].valid;
	       	if (wantedType != 0) segmentArray[i].crit = segmentArray[i].crit*(segmentArray[i].type == wantedType);
	}
	qsort(segmentArray,numSegments,sizeof(SSegment),compareSegments);
	for (int i = 0;i<numSegments;i++)
	{
		if (segmentArray[i].crit == 0){ 
			numSegments = i; 
			break;
		}
	}
	int yBrickRowThreshold = 100;
	for (int i = 1;i<numSegments;i++){
		if (segmentArray[0].y - segmentArray[i].y > yBrickRowThreshold){
			numSegments = i; 
			break;
		} 
	}
	
	for (int i = 0;i<numSegments;i++){
		float dX = (segmentArray[i].x-priorPosition.x);
		float dY = (segmentArray[i].y-priorPosition.y);
		segmentArray[i].crit = 10000-sqrt(dX*dX+dY*dY);
		printf("Segment %i: %f %f %f %f\n",i,segmentArray[i].x,segmentArray[i].y,priorPosition.x, priorPosition.y);
	}
	
	int XX,YY;
	for (int i = 0;i<numSegments;i++){
		for (int cn = 0;cn<4;cn++){
			for (int k = -3;k<=3;k++){
				for (int l = -3;l<=3;l++){
					YY = segmentArray[i].cornerY[cn]+k;
					XX = segmentArray[i].cornerX[cn]+l;
					if ( XX > 0 && XX < width && YY>0 && YY< height ) image->data[YY*width+XX] = -segmentArray[i].type;
					if (i == 0){
						XX = segmentArray[i].x+k;
						YY = segmentArray[i].y+l;
						if ( XX > 0 && XX < width && YY>0 && YY< height ) image->data[YY*width+XX] = -segmentArray[i].type;
					}
				}
			}
		}
	}

	for (int i = 0;i<numSegments;i++) printf("Segment %i size %i %i : %f %f %f %f\n",i,segmentArray[i].size,segmentArray[i].type,segmentArray[i].x,segmentArray[i].y,segmentArray[i].z,segmentArray[i].angle);

	//Seradi segmenty podle velikosti
	free(buffer);
	free(stack);
	free(contour);
	return segmentArray[0];
}
