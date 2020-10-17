#include "main.h"
#pragma once
#include "motorfunctions.hpp"
//#include "pid.hpp"
//#include "globalvar.hpp"
//#include "utilityfunctions.hpp"
using namespace pros;


struct odometrycontroller{
  ADIEncoder* left;
  ADIEncoder* right;
  ADIEncoder* back;
  double ds, db; //offsets from center of rotation, not diameter
  odometrycontroller(ADIEncoder en[],double s, double b):ds(s),db(b){left = &en[0]; right = &en[1]; back = &en[2];}
  //below: lazier, revised edition. Seperates into local and global coordinate conversions instead of all in one
  void posupdv2(){
    double xLN, yLN;
    double LD = rottodist(degtorad(left->get_value()),STD_TWHEEL_RADIUS);
    double RD = rottodist(degtorad(right->get_value()),STD_TWHEEL_RADIUS);
    double HD = rottodist(degtorad(back->get_value()),STD_BTWHEEL_RADIUS);
    double rang = ((RD-LD)/(ds*2));
    if (rang == 0){
      xLN = HD;
      yLN = RD;
    }
    else {
      yLN = 2*sin(rang/2)*(LD/rang + ds);
      xLN = 2*sin(rang/2)*(HD/rang + db);
    }
    double avang = angleG+(rang/2);
    double xC = yLN*cos(avang)+xLN*cos(avang-(M_PI/2)); //conversion to global coords
    double yC = yLN*sin(avang)+xLN*sin(avang-(M_PI/2));
    xG+=xC;
    yG+=yC;
    angleG+=rang;
    if (angleG > (M_PI*2)) angleG = angleG - (M_PI*2);
    if (angleG < 0) angleG = angleG + (M_PI*2);
    estspd = sqrt(xLN*xLN + yLN*yLN)*100; //x100 to convert to in/s from in/10ms
    if (xC != 0) heading = fmod(atan2(yC,xC),(2*M_PI));
    else if (yC > 0) heading = M_PI/2; //if moving directly up
    else if (yC < 0) heading = (3*M_PI)/2; //if moving directly down
    else heading = 0; //if not moving
    left->reset(); //these resets dont seem to be reliable, so we may have to resort to storing the pre update value
    right->reset();
    back->reset();
  }
  //keys position to a hardcoded target, use for wall allignments
  void key_position(double x, double y, double r){
    left->reset(); //these resets dont seem to be reliable, so we may have to resort to storing the pre update value
    right->reset();
    back->reset();
    xG = x;
    yG = y;
    angleG = r;
  }
};

/*coordcontroller: a wrapper for basecontroller to intepret coordinate grid inputs
    While it 100% is kinda stupid to hsave this many layers, this is done to allow
    a bit of distinction between each layer of sortware interaction. This way,
    troubleshooting, as well as understanding the code can be a bit easier.
*/
struct coordcontroller{
  basecontroller* mBase;
  PID* axiscontrollers; //the initial plan called for 3 PID controllers to allow for smooth motion curves, but for now we have a direct line approach
  //double* xyaT; //we are gonna try a potentially stupid approach, where we dont call coordcontroller but instead change the tgt coords directly
  coordcontroller(basecontroller a, PID b[]){mBase = &a; axiscontrollers = b;}
  /*returns true when target is reached
    potential camera implementation: overload update with version that replaces r and perp with camera controls
    this overload would input the desired color profile that the camera is looking for.
    note that constructor must be updated for this*/
  bool update(){
    //double yO = 0;
    //note that it isnt really nescessary, but made to minimize the risk of swaying in circles, it itself is disabled
    //past a certain point for safety's sake, although it is likely isn't gonna do anything weird when we get close to the target
    double xGD = (xyaT[0]-xG); //global x distance
    double yGD = (xyaT[1]-yG); //global y distance
    double dist = sqrt(xGD*xGD+yGD*yGD);
    double xD = 0;
    double yD = 0;
    double rD = 0; //VERY janky figure out better solution than a hard multiplier
    //we switch modes into a direct axis specific PID mode once we get close to prevent circular movement
    //this if statement can be optimized to just overwrite the GD variables instead of making the updvals, but this is more readable
    if (dist < 2.5){ //trigger x-y specific PID on activation
      xD = xGD*cos(getrelrad(angleG-M_PI/2,0))+yGD*cos(getrelrad(angleG,M_PI)); //relative distances to target
      yD = yGD*sin(getrelrad(angleG,M_PI))+xGD*sin(getrelrad(angleG-M_PI/2,0)); //relative distances to target
      rD = axiscontrollers[1].update(-7.5*(getrelrad(angleG,xyaT[2])));
    }else{
      double updXval = axiscontrollers[4].update(-xGD); //neg b/c PID responds to offset to target, not other way around
      double updYval = axiscontrollers[5].update(-yGD);
      xD = updXval*cos(getrelrad(angleG-M_PI/2,0))+updYval*cos(getrelrad(angleG,M_PI));
      yD = updYval*sin(getrelrad(angleG,M_PI))+updXval*sin(getrelrad(angleG-M_PI/2,0));
      rD = axiscontrollers[1].update(-20*(getrelrad(angleG,xyaT[2])));
    }

    //if ((sqrt(pow(xD,2)+pow(yD,2))) > 10) yO = axiscontrollers[3].update(getrelrad(heading, atan2(xG-xyaT[0],yG-xyaT[1])));
    //PID offset system if the motors aren't 100% correct orientation wise. May cause potential spinning issues near target
    //Below: Sketchy, and most likely redundent math to account for yO in the local coordinate system
    //xD+=yO*sin(atan2(xD,yD));
    //yD+=yO*cos(atan2(xD,yD));
    if(isnanf(rD)) rD = 0;
    double LPID = fabs(axiscontrollers[0].update(dist));
    double RPID = fabs(rD);
    double speed = determinesmallest(70, LPID+RPID);
    lcd::print(3,"Speed: %f",speed);
    lcd::print(4,"dist: %f", dist);
    lcd::print(5,"linear PID: %f", LPID);
    lcd::print(6,"rotational PID: %f", RPID);
    mBase->vectormove(xD,yD,rD,speed);
        //less than 2 inch distance, and less than 2% angle offset to commit to next stage
    if (round(dist/2 + fabs(rD/M_PI)*50) == 0) return true;
    else return false;
  }

  //this variation is for usage with motionpaths, where axiscontrollers merely maintains the speed target given by TSP
  bool update(double TSP){
    double yO = 0;
    //note that it isnt really nescessary, but made to minimize the risk of swaying in circles, it itself is disabled
    //past a certain point for safety's sake, although it is likely isn't gonna do anything weird when we get close to the target
    double xD = (xG-xyaT[0])*sin(angleG)+(yG-xyaT[1])*cos(angleG); //relative distances to target
    double yD = (yG-xyaT[1])*sin(angleG)+(xG-xyaT[0])*cos(angleG); //relative distances to target
    //unsure about recent correction from sin(angleG-pi/2) to cos(angleG), the thing is inversed but my initial math is probably wrong
    double rD = getrelrad(angleG,xyaT[2]); //VERY janky pls confirm if getrelrad works
    if ((sqrt(pow(xD,2)+pow(yD,2))) > 5) yO = axiscontrollers[2].update(getrelrad(heading, atan2(xG-xyaT[0],yG-xyaT[1])));
    //PID offset system if the motors aren't 100% correct orientation wise. May cause potential spinning issues near target
    //Below: Sketchy, and most likely redundent math to account for yO in the local coordinate system
    xD+=yO*sin(atan2(xD,yD));
    yD+=yO*cos(atan2(xD,yD));
    mBase->vectormove(xD,yD,rD,
      //above: unsure about subtracting yO or adding it
      //below: we do fabs because basecontroller already handles backwards vectors, so reversing power is useless
      fabs(axiscontrollers[0].update(TSP))+
      fabs(axiscontrollers[1].update(rD))
    );
    if (fabs(axiscontrollers[0].update(TSP))+fabs(axiscontrollers[1].update(rD)) < 15) return true;
    return false;
  }
};

/*motionpath:
    The goal of this is to implement movement paths through multiple points without
    stopping

    The current method of using a global trajectory array is kinda sketchy to use
    if we are gonna end up using this approach, but it's probably doable.
    The trajectory array is going to become an imaginary target though.

    The code design for this will have 3 development stages
      1. Converting PID speeds into percentage points for S curves, and making
        the bot not stop after each curve by making the PID percentage be over
        the entire trajectory
          This will be done by either:
            - Making xyaT into a bait coordinate, and getting axiscontrollers to
            automatically convert to percentage inside coordcontroller by extending
            xyaT percent units ahead in the ideal(line) direction (only good for 1.)
            - Make low key motion profiling and then turn the axis controller from being
            the primary source of movement into something used to maintain the profiling
            speed targets
      2. Adding a speed control when turning corners to minimize shaking from inertia(we might skip this)
      3. Making a curved transision between point to basically eliminate shaking

      Current implementation: step 1 done, low key motion profiling implementation
*/
struct motionpath{
  coordcontroller* controller;
  double** points; //not sure about how to make a pointer for this 2d array case
  dualScurve* tcurve;
  double percentage = 0;
  double oldlen = 0;
  int Tindex = 0;
  bool isfullycomplete = false;
  motionpath(coordcontroller b, double** loc){controller = &b; points = loc;
    oldlen = sqrt(pow(xG-points[Tindex][0],2) + pow(xG-points[Tindex][1],2));
  }
  bool update(){
    double TSP = points[Tindex][3];
    double newlen = sqrt(pow(xG-points[Tindex][0],2) + pow(xG-points[Tindex][1],2));
    percentage = ((newlen/oldlen)*100);
    if (Tindex == sizeof(points)-1 && percentage >= 50) TSP = tcurve->getval(percentage);
    if (Tindex == 0 && percentage <= 50) TSP = tcurve->getval(percentage);
    if (controller->update(TSP)){
      Tindex++;
      if (Tindex == sizeof(points)) isfullycomplete = true;
      oldlen = sqrt(pow(xG-points[Tindex][0],2) + pow(xG-points[Tindex][1],2));
      return true;
    }
    return false;
  }
};

/*motion:
  This is here so we can make a single std::vector for both
  vision sensor PID control, as well as bezier curve moves, and
  whatever else we may come up with, all we gotta do is extend this to it,
  and make it polymorphic */
class motion{

};
