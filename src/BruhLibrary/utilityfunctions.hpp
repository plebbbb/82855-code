#include "main.h"
#pragma once
//********************************************************************************//
/*UTLITITY FUNCTIONS*/

//for some reason, sort() is broken so i have to diy something,
/*NOTE: USING THIS IS ULTRA RISKY DUE TO DIRECTLY SORTING THE ARR
MAKE A NON POINTER COPY AND INSERTIONSORT THAT INSTEAD*/
extern void insertionsort(double arr[]);

//determinebiggest: returns biggest number, not absolute
extern double determinebiggest(double a, double b);

//isposorneg: returns 1 or -1 depending on if the value is positive or not
extern double isposorneg(double input);

//getrelrad: assumes positive radians, whereby 90deg right is 0
extern double getrelrad(double crad, double trad);

//rottodist: converts radians into distance based on the radius of rotator
extern double rottodist(double rad, double radius);

//degtorad: for user convience - converts degrees into radians
extern double degtorad(double deg);

//factorial
extern double factorial(double n);

//determinesmallest: determines smallest val, not absolute
extern double determinesmallest(double a, double b);

//debug feature for odometry
extern void odometrycontrollerdebug();
