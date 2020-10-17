#include "main.h"
#include "BruhLibrary/global.hpp"
using namespace pros;
void initialize() {
  lcd::initialize();
  //coordcontroller mover(base,bPID,xyaT);
  delay(100);
  //autonselection(); //calls auton selection method
}
//void disabled() {}
//void competition_initialize() {}
