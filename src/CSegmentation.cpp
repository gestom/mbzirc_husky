#include "CSegmentation.h"

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

int compareSegments(const void* m1,const void* m2)
{
	if (((SSegment*)m1)->size <  ((SSegment*)m2)->size) return -1;
	if (((SSegment*)m1)->size >  ((SSegment*)m2)->size) return 1;
	return 0;
}

//Vymazani promennych a jejich nastaveni na pocatecni hodnoty
CSegmentation::CSegmentation()
{
	debug = true;
	drawSegments = true;
}

CSegmentation::~CSegmentation()
{
}

SSegment CSegmentation::getSegment(int type,int number)
{
	SSegment result;
	result.x = segmentArray[number].x;
	result.y = segmentArray[number].y;
	return result;
}

SSegment CSegmentation::findSegment(CRawDepthImage *image,int minSize,int maxSize)
{
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
					if (buffer[pos] != 0) nncount++;
					if (buffer[pos] < 0){
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
				long long int cx,cy,sx,sy,sz;
				long long int cxx,cxy,cyy; 
				int maxX,maxY,minX,minY;
				maxX=maxY= -1;
				minX=minY = width*height;
				cxx=cxy=cyy=sx=sy=0;
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
				printf("%i %f %f %f\n",numSegments-1,fsx,fsy,fsz);
				segmentArray[numSegments-1].x = fsx;
				segmentArray[numSegments-1].y = fsy;
				segmentArray[numSegments-1].z = fsz;
				segmentArray[numSegments-1].minX = minX; 
				segmentArray[numSegments-1].minY = minY; 
				segmentArray[numSegments-1].maxX = maxX; 
				segmentArray[numSegments-1].maxY = maxY; 
				segmentArray[numSegments-1].roundness = M_PI*4*eigvl1*eigvl0/queueEnd;
				segmentArray[numSegments-1].circularity = eigvl1/eigvl0;

				/*calculate and publish corner candidates*/
				int corners = 0;
				int cX[4];
				int cY[4];
				float dist;
				float maxDist = 0;

				for (int cn = 0;cn<4;cn++){
					maxDist = 0;
					for (int s = 0;s<contourPoints;s++)
					{
						pos = contour[s];
						cx = pos%width; 
						cy = pos/width;
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
							maxDist = dist;
						}
					}
					for (int k = -3;k<=3;k++){
						for (int l = -3;l<=3;l++){
							image->data[(cY[cn]+k)*width+cX[cn]+l] = -1;
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
				segmentArray[numSegments-1].combo = 1; 
				segmentArray[numSegments-1].valid = 1; 
				for (int ii = 0;ii<4;ii++){
					segmentArray[numSegments-1].cornerX[ii] = cX[ii]+fsx;//-minX;
					segmentArray[numSegments-1].cornerY[ii] = cY[ii]+fsy;//-minY;
				}
			}else{
				numSegments--;
			}
		}
	}
	for (int i = 0;i<numSegments;i++) printf("Segment %i size %i: %f %f %f\n",i,segmentArray[i].size,segmentArray[i].x,segmentArray[i].x,segmentArray[i].y,segmentArray[i].z);
	//Seradi segmenty podle velikosti
	qsort(segmentArray,numSegments,sizeof(SSegment),compareSegments);
	free(buffer);
	free(stack);
	free(contour);
	return segmentArray[0];
}
