#pragma once

#include "vex.h"

// Centralize the one-and-only hardware instances so every module shares the
// same devices and there is a single place to change ports or motor inversion.
extern vex::competition Competition;
extern vex::brain Brain;
extern vex::controller Controller1;

extern vex::motor Pos_Y;
extern vex::motor Pos_X;
extern vex::motor Neg_Y;
extern vex::motor Neg_X;
extern vex::inertial HeadingSensor;
