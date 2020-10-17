#include "main.h"
#pragma once
/*it's throwing code errors but thats cuz we have a screwed up header structure where
if we include this into the main stream with global.hpp, we would have a recursive loop
of hpp files
it works in the actual code b/c we include it after global.hpp in main.cpp those*/
//if u wanna temparaly remove those errors so you can check for actual logic issues,
//uncomment the line below:
//#include "global.hpp"
//this is ripped straght from the old code. Ik it works but its still super sketchy
//also we may want to work on the delays
/*void autonselection() {
  ctrl.clear();
  while (true) {
    delay(50);
    if (!confirmedauton) {
      if (ctrl.get_digital_new_press(DIGITAL_LEFT)) {
        selectedauton--;
      } else if (ctrl.get_digital_new_press(DIGITAL_RIGHT)) {
        selectedauton++;
      } else if (ctrl.get_digital_new_press(DIGITAL_A))
        confirmedauton = true;
      if (selectedauton >= 5)
        selectedauton = 0;
      if (selectedauton < 0)
        selectedauton = 4;
      //    if (selectedauton != lastselectedauton){
      //  delay(50);
      switch (selectedauton) {}
      delay(50);
      //  }
    }
    if (confirmedauton) {
      ctrl.clear();
      delay(50);
      ctrl.set_text(0, 0, "AUTON SELECTED");
      delay(50);
      ctrl.set_text(1, 0, "PRESS B TO confirm");
      delay(50);
      ctrl.set_text(2, 0, "PRESS A TO CANCEL");
      if (ctrl.get_digital_new_press(DIGITAL_A)) {
        ctrl.clear();
        delay(50);
        ctrl.set_text(0, 0, "SELECT AUTON");
        delay(50);
        ctrl.set_text(2, 0, "PRESS A TO CONFIRM");
        delay(50);
        confirmedauton = false;
      }
      if (ctrl.get_digital_new_press(DIGITAL_B))
        break;
    }
    delay(50);
  }
  ctrl.clear();
  ctrl.set_text(0, 0, "PROGRAM START"); //
}*/
