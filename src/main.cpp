/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       IsitT                                                     */
/*    Created:      4/7/2026, 8:50:57 AM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;
brain Brain;
controller Controller1;

// define your global instances of motors and other devices here
motor Pos_Y(PORT1, gearSetting::ratio18_1, true);
motor Pos_X(PORT3, gearSetting::ratio18_1, true);
motor Neg_Y(PORT7, gearSetting::ratio18_1, false);
motor Neg_X(PORT5, gearSetting::ratio18_1, false);
// Change PORT9 if your inertial sensor is plugged into a different port.
inertial HeadingSensor(PORT9);

namespace
{

  constexpr int kDriverLoopMs = 20;
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDeadbandPct = 5.0;
  constexpr double kHeadingStickDeadbandPct = 20.0;
  constexpr double kHeadingHoldKp = 0.85;
  constexpr double kMaxHeadingHoldPct = 60.0;
  constexpr double kHeadingHoldEngageErrorDeg = 1.0;

  // These aliases define the physical wheel order for the X-drive mix.
  // If translation or heading are mirrored on the robot, update this mapping
  // or the motor reversed flags instead of changing the control math.
  motor &FrontLeftMotor = Pos_Y;
  motor &FrontRightMotor = Neg_X;
  motor &BackLeftMotor = Pos_X;
  motor &BackRightMotor = Neg_Y;

  double desiredHeadingDeg = 0.0;
  bool headingSensorReady = false;
  bool wasResetPressed = false;

  double clampValue(double value, double minValue, double maxValue)
  {
    if (value < minValue)
    {
      return minValue;
    }

    if (value > maxValue)
    {
      return maxValue;
    }

    return value;
  }

  double wrapDegrees180(double degrees)
  {
    while (degrees > 180.0)
    {
      degrees -= 360.0;
    }

    while (degrees <= -180.0)
    {
      degrees += 360.0;
    }

    return degrees;
  }

  double wrapDegrees360(double degrees)
  {
    while (degrees >= 360.0)
    {
      degrees -= 360.0;
    }

    while (degrees < 0.0)
    {
      degrees += 360.0;
    }

    return degrees;
  }

  double applyDeadband(double value, double deadbandPct)
  {
    return std::fabs(value) < deadbandPct ? 0.0 : value;
  }

  void spinDrive(double frontLeft, double frontRight, double backLeft,
                 double backRight)
  {
    const double maxMagnitude = std::fmax(
        std::fmax(std::fabs(frontLeft), std::fabs(frontRight)),
        std::fmax(std::fabs(backLeft), std::fabs(backRight)));

    if (maxMagnitude > 100.0)
    {
      const double scale = 100.0 / maxMagnitude;
      frontLeft *= scale;
      frontRight *= scale;
      backLeft *= scale;
      backRight *= scale;
    }

    FrontLeftMotor.spin(fwd, frontLeft, pct);
    FrontRightMotor.spin(fwd, frontRight, pct);
    BackLeftMotor.spin(fwd, backLeft, pct);
    BackRightMotor.spin(fwd, backRight, pct);
  }

  void stopDrive(brakeType mode = brakeType::brake)
  {
    FrontLeftMotor.stop(mode);
    FrontRightMotor.stop(mode);
    BackLeftMotor.stop(mode);
    BackRightMotor.stop(mode);
  }

  void calibrateHeadingSensor()
  {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    if (!HeadingSensor.installed())
    {
      Brain.Screen.print("Missing inertial on PORT9");
      headingSensorReady = false;
      return;
    }

    Brain.Screen.print("Calibrating inertial...");

    HeadingSensor.calibrate();
    while (HeadingSensor.isCalibrating())
    {
      wait(50, msec);
    }

    HeadingSensor.setHeading(0.0, degrees);
    desiredHeadingDeg = 0.0;
    headingSensorReady = true;

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Inertial ready");
  }

} // namespace
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  calibrateHeadingSensor();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void)
{
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void)
{
  if (headingSensorReady)
  {
    desiredHeadingDeg = HeadingSensor.heading(degrees);
  }

  // User control code here, inside the loop
  while (1)
  {
    const double fieldForward =
        applyDeadband(Controller1.Axis3.position(pct), kDeadbandPct);
    const double fieldRight =
        applyDeadband(Controller1.Axis4.position(pct), kDeadbandPct);
    const double headingForward =
        applyDeadband(Controller1.Axis2.position(pct), kHeadingStickDeadbandPct);
    const double headingRight =
        applyDeadband(Controller1.Axis1.position(pct), kHeadingStickDeadbandPct);
    const double headingDeg =
        headingSensorReady ? HeadingSensor.heading(degrees) : 0.0;

    if (headingSensorReady && (headingForward != 0.0 || headingRight != 0.0))
    {
      desiredHeadingDeg = wrapDegrees360(
          std::atan2(headingRight, headingForward) * 180.0 / kPi);
    }

    const bool resetPressed = Controller1.ButtonA.pressing();
    if (headingSensorReady && resetPressed && !wasResetPressed)
    {
      HeadingSensor.setHeading(0.0, degrees);
      desiredHeadingDeg = 0.0;
    }
    wasResetPressed = resetPressed;

    double turnCommand = 0.0;
    if (headingSensorReady)
    {
      const double headingErrorDeg = wrapDegrees180(desiredHeadingDeg - headingDeg);
      if (std::fabs(headingErrorDeg) > kHeadingHoldEngageErrorDeg)
      {
        turnCommand = clampValue(headingErrorDeg * kHeadingHoldKp,
                                 -kMaxHeadingHoldPct,
                                 kMaxHeadingHoldPct);
      }
    }
    else
    {
      turnCommand = applyDeadband(Controller1.Axis1.position(pct), kDeadbandPct);
    }

    const double headingRad = headingDeg * kPi / 180.0;
    const double robotForward = headingSensorReady
                                    ? fieldForward * std::cos(headingRad) +
                                          fieldRight * std::sin(headingRad)
                                    : fieldForward;
    const double robotRight = headingSensorReady
                                  ? -fieldForward * std::sin(headingRad) +
                                        fieldRight * std::cos(headingRad)
                                  : fieldRight;

    const double frontLeftCommand = robotForward + robotRight + turnCommand;
    const double frontRightCommand = robotForward - robotRight - turnCommand;
    const double backLeftCommand = robotForward - robotRight + turnCommand;
    const double backRightCommand = robotForward + robotRight - turnCommand;

    if (fieldForward == 0.0 && fieldRight == 0.0 && turnCommand == 0.0)
    {
      stopDrive(brakeType::brake);
    }
    else
    {
      spinDrive(frontLeftCommand, frontRightCommand, backLeftCommand,
                backRightCommand);
    }

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Heading: %.1f", headingDeg);
    Brain.Screen.setCursor(2, 1);
    if (headingSensorReady)
    {
      Brain.Screen.print("Target:  %.1f", desiredHeadingDeg);
    }
    else
    {
      Brain.Screen.print("No inertial fallback");
    }

    wait(kDriverLoopMs, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true)
  {
    wait(100, msec);
  }
}
