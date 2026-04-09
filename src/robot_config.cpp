#include "robot_config.h"

vex::competition Competition;
vex::brain Brain;
vex::controller Controller1;

vex::motor Pos_Y(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor Pos_X(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor Neg_Y(vex::PORT7, vex::gearSetting::ratio18_1, false);
vex::motor Neg_X(vex::PORT5, vex::gearSetting::ratio18_1, false);
vex::inertial HeadingSensor(vex::PORT9);
