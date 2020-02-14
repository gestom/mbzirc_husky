#ifndef ORDER_H
#define ORDER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/**
@author Tom Krajnik
*/

class CTSP
{
public:
    CTSP(float ix[],float iy[],int len);
    ~CTSP();

    void solve(int iterations);
    void print();
    void save(const char* name);
    float *x;
    float *y;

private:
    /*reverse part of the circuit (from the a-th to the b-th node)*/
    void swap(int a,int b);

    /*calculate intersection of two lines*/
    bool intersect(float ax1,float ay1,float ax2,float ay2,float bx1,float by1,float bx2,float by2);

    bool intersectSet(int *a,int *b);

    /*remove intersections*/
    void interswap();

    /*simulated annealing*/
    void randomswap();

    /*get path length*/
    float getLength();


    int len;
    bool debug;
};
#endif

