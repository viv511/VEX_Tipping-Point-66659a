#include "main.h"

using namespace pros;
#ifndef GLOBALS
#define GLOBALS

extern pros::Motor FL;
extern pros::Motor FR;
extern pros::Motor ML;
extern pros::Motor MR;
extern pros::Motor BL;
extern pros::Motor BR;
extern pros::Motor ring;
extern pros::Motor arm;
//controller
extern pros::Controller controller;
//sensor
extern pros::Imu inertial;
//pistons
extern pros::ADIDigitalOut pistonF;
extern pros::ADIDigitalOut pistonB1;
extern pros::ADIDigitalOut pistonB2;

#endif
