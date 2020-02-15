#include "order.h"

CTSP::CTSP(float ix[],float iy[],int leni)
{
	len = leni;
	x = (float*)malloc((len+1)*sizeof(float));
	y = (float*)malloc((len+1)*sizeof(float));
	memcpy(x,ix,(len+1)*sizeof(float));
	memcpy(y,iy,(len+1)*sizeof(float));
	debug = false;
}

CTSP::~CTSP()
{
	free(x);
	free(y);
}

void CTSP::swap(int a,int b)
{
	float xa[len];
	float ya[len];
	memcpy(xa,x,len*sizeof(float));
	memcpy(ya,y,len*sizeof(float));
	if (debug){
		for (int i = 0;i<a;i++) printf("%i ",i);
		for (int i = a;i<=b;i++) printf("%i ",b-i+a);
		for (int i = b+1;i<len;i++) printf("%i ",i);
	}
	for (int i = a;i<=b;i++){
	       	x[i]=xa[b-i+a];
	       	y[i]=ya[b-i+a];
	}
	if (debug) printf("\n");
	//print();
}

bool CTSP::intersect(float ax1,float ay1,float ax2,float ay2,float bx1,float by1,float bx2,float by2)
{
	float a1 = ay2-ay1;
	float b1 = ax1-ax2;
	float c1 = a1*ax1+b1*ay1;

	float a2 = by2-by1;
	float b2 = bx1-bx2;
	float c2 = a2*bx1+b2*by1;
	
	double det = a1*b2 - a2*b1;
	if (det == 0) return false; //lines paralel
	float X = (b2*c1 - b1*c2)/det;
	float Y = (a1*c2 - a2*c1)/det;
	if (debug) printf("Intersect at %.0f,%.0f-%.0f,%.0f  %.0f,%.0f-%.0f,%.0f: %.1f %.1f\n", ax1, ay1, ax2, ay2, bx1, by1, bx2, by2,X,Y);
	return  ((fmin(ay1,ay2) <= Y) && (Y <= fmax(ay1,ay2)) && (fmin(ax1,ax2) <=X) && (X <= fmax(ax1,ax2)) && (fmin(by1,by2) <= Y) && (Y <= fmax(by1,by2)) && (fmin(bx1,bx2) <=X) && (X <= fmax(bx1,bx2))); 
}

bool CTSP::intersectSet(int *a,int *b)
{
	for (int i=0;i<len;i++)
	{
		for (int j=i+2;j<len;j++)
		{
			if (intersect(x[i],y[i],x[i+1],y[i+1],x[j],y[j],x[j+1],y[j+1]))
			{
				*a = i+1;
				*b = j;
				if (debug)printf("Intersect %i %i\n",*a,*b);
				return true;
			} 
		}
	}
	return false;
}

float CTSP::getLength()
{
	float d = 0;
	float dx,dy;
	for (int i = 0;i<len;i++){
	       	dx = x[i]-x[i+1];
		dy = y[i]-y[i+1];
		d+=sqrt(dx*dx+dy*dy);
	}
	return d;
}

void CTSP::save(const char *name)
{
	FILE *file = fopen(name,"w");
	for (int i = 0;i<len+1;i++)fprintf(file,"%02.0f %02.0f\n",x[i],y[i]);
	fclose(file);
}

void CTSP::print()
{
	for (int i = 0;i<len+1;i++)printf("%02.0f %02.0f\n",x[i],y[i]);
}

void CTSP::randomswap()
{
	float xa[len];
	float ya[len];
	memcpy(xa,x,len*sizeof(float));
	memcpy(ya,y,len*sizeof(float));
	float minLength = getLength();
	int a,b;
	for (int i = 0;i<1000;i++)
	{
		a = rand()%(len-2)+1;
		b = rand()%(len-2)+1;
		if (a<b) swap(a,b);
		if (getLength() < minLength)
		{
			memcpy(xa,x,len*sizeof(float));
			memcpy(ya,y,len*sizeof(float));
			minLength = getLength();
		} 
		memcpy(x,xa,len*sizeof(float));
		memcpy(y,ya,len*sizeof(float));
	}
}

void CTSP::interswap()
{
	int a,b;
	int numSwaps=0;
	while (intersectSet(&a,&b) && numSwaps<100)
	{
	       	swap(a,b);
		numSwaps++;
		if (debug) printf("%f\n",getLength());
	}
}

void CTSP::solve(int iter)
{
	if (len > 2){
		for (int i = 0;i<iter;i++)
		{
			randomswap();
			interswap();
		}
	}
}

//int main(int argc,char* argv[])
//{
//	float x[500];
//	float y[500];
//	int len = 0;
//	FILE *file = fopen(argv[1],"r");
//	while (feof(file)==0)
//	{
//		fscanf(file,"%f %f\n",&x[len],&y[len]);
//		len++;
//	}
//	fclose(file);
//	CTSP tsp(x,y,len);
//	tsp.solve(len*2);
//	tsp.save(argv[2]);
//	return 0;
//}
