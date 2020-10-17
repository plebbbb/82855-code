#include "main.h"
#include "BruhLibrary/global.hpp"

/*
A guide to the current autonomous approach
  1. Commands are to be updated via adjusting positional variables
  2. There is to be no direct interfacing with class functions

The idea is, we use positional variables, which the wrappers are pointed to
There really isn't an advantage to doing stuff like this, I just thought it looked
cool. In the 65% chance this approach is risky, we can switch to a function based
system

*/

//********************************************************************************//
/*auton procedures:
procedure documentation:
  x coord, y coord, angle target... TBA
*/

//postionsetTEST: Benchmark test to ensure the functionality of the motors
double positionsetTEST[][3] = {
  {0,0,M_PI/2},
  {30,15,M_PI},
  {10,10,(3*M_PI)/2},
  {20,25,(M_PI)/5},
  {0,30,M_PI*2},
  {0,0,M_PI/2}
};

//********************************************************************************//
void autonomous(){
  int cycle = 0;
  xyaT[0] = positionsetTEST[0][0];
  xyaT[1] = positionsetTEST[0][1];
  xyaT[2] = positionsetTEST[0][2];
  coordcontroller mover(base,bPID);
//  odometrycontrollerdebug();
  while(true){
    odo.posupdv2();
    //the idea for this if statement is that it calls all updates
    //and only passes once everything is done, before updating the variables
    odometrycontrollerdebug();
    if (mover.update()/* && (motorf object).PID_UPDATE_CYCLE ....*/ && cycle < sizeof(positionsetTEST)/sizeof(*positionsetTEST)-1){
      /*TBD: Make the movement array into a 3d array,
        have the third dimension be for each different auton
        and the */
      //is the for loop cheese? theres the something:something way but
      //idk how that works, nor if its even in C++
      cycle = cycle+1;
      xyaT[0] = positionsetTEST[cycle][0];
      xyaT[1] = positionsetTEST[cycle][1];
      xyaT[2] = positionsetTEST[cycle][2];
      /*while the goal was to not use functions, chances are we are gonna
      have to for motorf because I really dont wanna rewrite those constructors*/
      //suggested code for doing so//
      /*
      (motorf object).PID_MOVE_TARGET(positionsetTEST[cycle][specific index]);
      */
    }
    lcd::print(7,"Cycle: %d", cycle);
    delay(10);
  }
}
