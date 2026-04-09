/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       IsitT                                                     */
/*    Created:      4/7/2026, 8:50:57 AM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "driver_control.h"
#include "robot_config.h"
#include "vex.h"

void pre_auton(void)
{
  initializeDriverControl();
}

void autonomous(void)
{
}

void usercontrol(void)
{
  runDriverControl();
}

int main()
{
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true)
  {
    vex::wait(100, vex::msec);
  }
}
