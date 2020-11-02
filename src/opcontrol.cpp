#include "main.h"
#include "BruhLibrary/global.hpp"
using namespace pros;
double ang = M_PI/2;

//*******************************************************************************//
//Control scheme configuration
//array format: left-right, forwards-back, clockwise-counterclockwise
/*controller_analog_e_t controlscheme[]{
  ANALOG_LEFT_X,
  ANALOG_LEFT_Y,
  ANALOG_RIGHT_X
};

//Control scheme featureset
//array format: enable relative mode, enable angle hold
bool configoptions[]{
  false,
  true
};*/

//Above: probably fixed the problem forcing this to be here, if not, above was a working option

//*******************************************************************************//
//The actual code
void opcontrol(){
  //TBD: fix the pointers on these so they actually work - probably done see older revisions for all the old stuff needed to get it to work
  while(true){
    odo.posupdv2();
    odometrycontrollerdebug();
    //useonlyinopcontrol.ssc->vectormove(10, 10, 0, 10);
    //lcd::print(1,"%f",useonlyinopcontrol.ssc->MAP[0].cosV);
    //useonlyinopcontrol.relativemove(ctrl.get_analog(ANALOG_RIGHT_X));
    useonlyinopcontrol.move();
    if (ctrl.get_digital_new_press(DIGITAL_B)) configoptions[0] = !configoptions[0];
    delay(10);
  }; /*
//below: test for autonomous
  while(true){
    odo.posupdv2();
    mover.update();
    odometrycontrollerdebug();
    delay(10);
  }*/
//  autonomous();
}
