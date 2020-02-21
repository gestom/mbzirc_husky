SSegment CSegmentation::findSeparatedSegment(Mat *image,Mat *coords,SSegment *output,int minSize,int maxSize)
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
        buffer[topPos*width+i] = borderType+1;
        buffer[bottomPos*width+i] =  borderType+2;
    }

    for (int i = topPos;i<bottomPos;i++){
        buffer[width*i+leftPos] =  borderType+3;
        buffer[width*i+rightPos] =  borderType+4;
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
	    segmentArray[numSegments-1].warningTop = 0; 
	    segmentArray[numSegments-1].warningBottom = 0; 
	    segmentArray[numSegments-1].warningLeft = 0; 
	    segmentArray[numSegments-1].warningRight = 0; 
            segmentArray[numSegments-1].x = i%width;
            segmentArray[numSegments-1].y = i/width;
            segmentArray[numSegments-1].valid = 0;
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
                    if (buffer[pos] > borderType){
			    if (buffer[pos] == borderType+1)segmentArray[numSegments-1].warningTop = 1;
			    if (buffer[pos] == borderType+2)segmentArray[numSegments-1].warningBottom = 1;
			    if (buffer[pos] == borderType+3)segmentArray[numSegments-1].warningLeft = 1;
			    if (buffer[pos] == borderType+4)segmentArray[numSegments-1].warningRight = 1;
			    segmentArray[numSegments-1].warning = 1;
		    }
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

    //Seradi segmenty podle velikosti
    for (int i = 0;i< numSegments;i++) segmentArray[i].crit = segmentArray[i].size;
    qsort(segmentArray,numSegments,sizeof(SSegment),compareSegments);
    biggest_segment = segmentArray[0];

//    separateContours(buffer,coords,output,minSize,maxSize);
    result = segmentArray[0];
    //vykreslime vysledek
    int j = 0;
    if (drawSegments){
        for (int i = 0;i<len;i++){
            j = buffer[i];
            if (j > 1000000) image->at<Vec3b>(i/width,i%width) = Vec3f(0,0,0);
        }
    }
    free(buffer);
    free(stack);
    free(contour);
    return result;
}
