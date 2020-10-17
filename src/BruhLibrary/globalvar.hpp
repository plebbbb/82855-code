#include "main.h"
using namespace pros;
#pragma once

//********************************************************************************//
/*GLOBAL DEFINITIONS*/
//in inches or rads

#define Y_AXIS_TWHEEL_OFFSET 6.95 //offset from center line of the y axis tracking sheel
#define X_AXIS_TWHEEL_OFFSET 11.9 //not being used currently. we'd need another horz wheel for that
#define STD_WHEEL_RADIUS 1.625 //4in base wheels
#define STD_TWHEEL_RADIUS 1.375 //2.75in wheel for left-right tracking wheels
#define STD_BTWHEEL_RADIUS 2.00 //4in wheel used on back wheel for testing bot
#define BASE_MOTOR_RPM 200 //base motor rpm


//********************************************************************************//
/*GLOBAL VARIABLES*/
//TBD - use std::vector when it's actually nescessary instead of spamming so many arrays

//V5 controller
extern Controller ctrl;

//Global angle, rads, positive
extern double angleG;

//Global x coordinate
extern double xG;

//Global y coordinate
extern double yG;

//Global relative changes from last cycle
extern double xR;
extern double yR;

//Array of target x, y, and angle
extern double xyaT[3]; //is making this an array a good idea or not tbh not sure

//Observed heading of robot, rads
extern double heading; //may end up useless

//Observed velocity of base
extern double estspd;

//global slowmode speed multiplier
extern double speedmultiplier;

//array of position sets for auton to reach, to be revised upon design completion
extern double positionsetTEST[][3];

//PID template values
extern double PIDKvals[][3];
extern bool PIDSvals[][3];
extern double PIDLvalues[][2];

//S curve template values
extern double Scurvevals[][4];

//auton selector values
extern bool confirmedauton;
extern int selectedauton;

//speed bezier curve transformation weight factor
extern double vscalefac;

//opcontrol configuration
extern controller_analog_e_t controlscheme[];
extern bool configoptions[];
