#include "BruhLibrary/global.hpp"

/*
A guide to the current autonomous approach
  1. Commands are to be updated via adjusting positional variables
  2. There is to be no direct interfacing with class functions

The idea is, we use positional variables, which the wrappers are pointed to
There really isn't an advantage to doing stuff like this, I just thought it looked
cool. In the 65% chance this approach is risky, we can switch to a function based
system

//note: old auton config info here has been depreciated. New auton commands in global.cpp, may decide to move it over.
//also, the old line testing code has been removed. See old commits for it, like pre october or something
/********************************************************************************/
void autonomous(){
  double perc = 0;
  int arr = 0;
  motionpaths[arr].computepath();
  while(true){
    odo.posupdv2();
    odometrycontrollerdebug();
    if (mover.updateMP()){
      if (motionpaths[arr].computepath() && arr < motionpaths.size()-1) arr += 1;
    }
    delay(10); //refresh clock
  }
}
