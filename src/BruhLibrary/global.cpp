#include "main.h"
#include "global.hpp"
// This cpp is for the declration of extern objects from global.hpp

/*a tip making motorw arrays:
  it is suggested to not inverse motor direction, and instead figure out the
  correct vector direction for positive movement. This prevents potential issues
  with rotation.
*/


//********************************************************************************//
//Variables:
double speedmultiplier = 100; //IN PERCENT, 100 being 100%
double angleG = M_PI/2;
double xG = 0;
double yG = 0;
double xR = 0;
double yR = 0;
double estspd = 0;
double heading = angleG;
double xyaT[3] = {0,0,angleG};
double vscalefac = 0;

//********************************************************************************//
//ADIEncoder arrays:
//ADIEncoder format: pin 1, pin2, inversed or not
//Array format: Left, Right, Back
ADIEncoder odencoders[3] = {
  ADIEncoder('C','D',true),
  ADIEncoder('B','A',false),
  ADIEncoder('F','E',true)
};


//********************************************************************************//
//motorw arrays:
//motorw format: pin, inv dir, orientation, wheel size
motorw kiwimotors[] = {
  motorw(1,true,(M_PI*2)/3), //right motor
  motorw(2,true,(M_PI*4)/3), //left motor
  motorw(3,true,0) //Pure translational Motor
};

motorw xdrivemotors[] = {
  motorw(15,true,(3*M_PI)/4), //top right corner
  motorw(16,true,(M_PI)/4), //bottom right corner
  motorw(17,true,(7*M_PI)/4), //bottom left corner
  motorw(19,true,(5*M_PI)/4), //top left corner
};

//********************************************************************************//
//PID Variables

//********************************************************************************//
//PIDKvals format: Pk, Ik, Dk
double PIDKvals[][3] = {
  {7.5,0.0005,2},     //direct distance PID
  {5,1.5,3},        //direct rotation PID for driver mode
  {3,0,2},       //direct rotation PID
  {2,0.05,1},       //heading offset PID
  {3,0,1},          //direct X/Y axis PID
};

//********************************************************************************//
/*PIDKvals format:
S-curve enable/disable,
fractional I enable/disable,
I hardstop at target enable/disable*/
bool PIDSvals[][3] = {
  {false,false,true}
};

//********************************************************************************//
//PIDL values: PID hard limit, I value hard limit
double PIDLvals[][2] = {
  {100,50},
  {50,50}
};

//********************************************************************************//
//S curve VARIABLES
//Scurve possible Y range: 0-127, X range: 0-50
//Get your coefficients from desmos! https://www.desmos.com/calculator/aydhkipdkz
//Index 0: max spd, Index 1: slope, Index 2: horizontal offset, Index 3: vertical offset
//max spd should be always positive!
double Scurvevals[][4] = {
  {127,0.2,24.2,0} //default settings from the desmos link
};

//S curves
curveS curves[] = {
  curveS(Scurvevals[0])
};

//dualScurve wrapper
dualScurve curvesets[] = {
  dualScurve(curves[0]) //default desmos link, flipped past 50%
};


//********************************************************************************//
//PID controllers
//constructor scheme: PIDKvals, PIDSvals, PIDLvals, (OPTIONAL)dualScurve

//PID for base navigation
PID bPID[] = {
  PID(PIDKvals[0],PIDSvals[0],PIDLvals[1]), //direct distance PID
  PID(PIDKvals[2],PIDSvals[0],PIDLvals[1]), //direct rotation PID
  PID(PIDKvals[1],PIDSvals[0],PIDLvals[0]), //direct rotation PID for driver mode
  PID(PIDKvals[3],PIDSvals[0],PIDLvals[0]), //heading offset PID
  PID(PIDKvals[4],PIDSvals[0],PIDLvals[0]), //direct X axis PID
  PID(PIDKvals[4],PIDSvals[0],PIDLvals[0]), //direct Y axis PID
};

PID e = PID(PIDKvals[0],PIDSvals[0],PIDKvals[0],curvesets[0]); //example setup for a motorF

//********************************************************************************//
/*
//Control scheme configuration
//array format: left-right, forwards-back, clockwise-counterclockwise
controller_analog_e_t controlscheme[]{
  ANALOG_LEFT_X,
  ANALOG_LEFT_Y,
  ANALOG_RIGHT_X
};

//Control scheme featureset
//array format: enable absolute mode, enable angle hold
bool configoptions[]{
  true,
  true
};

*/
//********************************************************************************//
//actual controllers
Controller ctrl = E_CONTROLLER_MASTER;
odometrycontroller odo(odencoders,Y_AXIS_TWHEEL_OFFSET,X_AXIS_TWHEEL_OFFSET);
basecontroller base(xdrivemotors);
//coordcontroller mover(base,bPID,xyaT);
//opcontrolcontroller useonlyinopcontrol(base,controlscheme,ctrl,bPID[2],configoptions);
//********************************************************************************//
//functions:
double determinebiggest(double a, double b){
  if (a>b) return a;
  return b;
}

double isposorneg(double input){
  //there is 100% a better solution but this works so imma keep it for now
  //if someone figures it out pls change it
  return input/fabs(input); //its supposted to return 1 or -1, fyi
}

//WARNING, CRAD AND TRAD MEAN NOTHING. TEST EACH IMPLEMENTATION
double getrelrad(double crad, double trad){
  //note: untested but probably correct. Pls test
/*  if (fabs(crad-trad) > M_PI) return -(2*M_PI-fabs(crad-trad))*isposorneg(crad-trad);
  return (crad-trad);*/
  //double scrad = fmod(crad,M_PI*2);
//  double strad = fmod(trad,M_PI*2);
  double rdval = (trad-crad);
  if (rdval < -M_PI) return rdval+(M_PI*2);
  if (rdval > M_PI) return -((M_PI*2)-rdval);
  return rdval;
}

//this is pretty stupid
double rottodist(double rad, double radius){
  return rad*radius;
}

//this is solely for convience -
//NGL tho for everyone who already has to use radians, just get used to them
double degtorad(double deg){
  return fmod(deg*(M_PI/180),2*M_PI);
};

//recursive approach to computing factorials
double factorial(double n){
  return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

//determine smallest functions
double determinesmallest(double a, double b){
  if (a > b) return b;
  return a;
}

//Below: doesnt work b/c header issues
/*void basecontrollerdebug(basecontroller a){
  lcd::print(0, "TR motor: %f",a.vals[0]);
  lcd::print(1, "BR motor: %f",a.vals[1]);
  lcd::print(2, "BL motor: %f",a.vals[2]);
  lcd::print(3, "TL motor: %f",a.vals[3]);
  lcd::print(5, "Rot Ratio: %f",a.rotationalratio);
}
void trackingwheeldebug(odometrycontroller a) {
lcd::print(0,"Left Encoder: %d", (a.left->get_value()));
lcd::print(1,"Right Encoder: %d", (a.right->get_value()));
lcd::print(2,"Back Encoder: %d", (a.back->get_value()));
}*/
void odometrycontrollerdebug(){
lcd::print(0,"X: %f",xG);
lcd::print(1,"Y: %f", yG);
lcd::print(2,"Angle: %f", angleG);
//lcd::print(3,"Est Spd: %d in/s", (int)estspd);
//lcd::print(4, "Est Heading: %f", heading);
}
