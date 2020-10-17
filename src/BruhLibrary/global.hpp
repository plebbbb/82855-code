#include "main.h"
//#include "globalvar.hpp"
//#include "utilityfunctions.hpp"
//#include "pid.hpp"
//#include "motorfunctions.hpp"
#include "navigationfunctions.hpp"
#include "userinterface.hpp"
#pragma once
using namespace pros;
/*
TO BE DONES:
  1. ORGANIZE EVERYTHING HERE INTO THEIR OWN RESPECTIVE .hpp files,
     I think we will keep initilizations centalized, maybe moved to main.cpp
     1.b - FIX THE ORGANIZATION STRUCTURE: it compiles fine but the source hpps
           think they are broken
           1.c - FIX THE HPP HIEARCHY: I dont think this strat can handle more
           than one slave hpp per hpp, so it ends up being pretty cheese
  2. FINISH ALL CLASS DEFINITIONS
*/

//This is basically a place to put all class objects
//********************************************************************************//

//Scurve instances
extern dualScurve curvesets[];
extern curveS curves[];

//PID template instances
extern PID bPID[];

//global hardware interface layers
extern odometrycontroller odo;
extern motorw kiwimotors[];
extern motorw xdrivemotors[];
extern basecontroller base;
extern coordcontroller mover;
extern opcontrolcontroller useonlyinopcontrol;
//TBD: add all the other controllers

//std::vector of processed position commands
extern std::vector<motion> processedpaths;
